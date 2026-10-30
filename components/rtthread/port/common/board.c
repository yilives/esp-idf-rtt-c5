#include <rtthread.h>
#include "sdkconfig.h"
#include "port_systick.h"
#include "riscv/interrupt.h"
#include "esp_heap_caps_init.h"
#include "esp_err.h"
#include "esp_private/startup_internal.h"

void rt_hw_board_init(void)
{
    /* startup.c:do_core_init() already initialized the RT-Thread heap.
     * Reinitializing it here would erase and reinsert its memory object.
     */

    /* rtthread_startup() has disabled MIE. Reuse IDF's OS tick alarm; the
     * first thread's mret enables interrupts after timer/scheduler init.
     * vectors_rtthread.S saves, raises and restores the interrupt threshold in
     * software on both C3 and C6, which do not define
     * SOC_INT_HW_NESTED_SUPPORTED.
     */
    vPortSetupTimer();
    esprv_int_set_threshold(RVHAL_INTR_ENABLE_THRESH);

#if defined(SOC_ESP32_S31)
    /* esp_timer has to be finished here, and the reason is a gap between two
     * decisions that were each correct on their own.
     *
     * esp_timer splits into two halves. esp_timer_early_init() only starts the
     * SYSTIMER counter, and it runs as a CORE-stage init function, so
     * esp_timer_get_time() already works. esp_timer_init() is the other half:
     * it creates the dispatch task and installs the alarm ISR, and it is reached
     * from ESP_SYSTEM_INIT_FN(esp_timer_init_os) - which IDF compiles out under
     * CONFIG_IDF_RTOS_RTTHREAD, because upstream assumes the RTOS brings its own
     * timers.
     *
     * RT-Thread does have its own timers, but they are not esp_timer, and
     * nothing in this port calls esp_timer_init(). The result is a driver that
     * links, runs, and then aborts on first use: esp_phy_enable() calls
     * phy_track_pll_init() unconditionally, that calls
     * ESP_ERROR_CHECK(esp_timer_create(...)), and esp_timer_create() returns
     * ESP_ERR_INVALID_STATE whenever s_timer_task is still NULL
     * (esp_timer.c:105). An ESP_ERROR_CHECK on a non-OK code aborts.
     *
     * So the missing half is supplied here. Called before the scheduler starts,
     * which is fine: rt_thread_startup() only puts the thread on the ready
     * queue. It is also safe to call once - init_timer_task() returns
     * ESP_ERR_INVALID_STATE if it has already run.
     *
     * Gated on the shim, because esp_timer's task is only needed by the IDF
     * components that run on it (esp_phy, esp_wifi). With the shim off nothing
     * in the image calls esp_timer_create(), so the task would be dead weight. */
    /* esp_timer_init() moved to a SECONDARY-stage init function; see below. */

    /* Register the pin device before the scheduler runs so anything that uses
     * rt_pin_*() during init finds it. Also the reference that keeps the GPIO
     * driver object from being dropped out of the component archive:
     * rt_device_pin_register() lives in the drivers archive, not here. */
    extern int rt_hw_pin_init(void);
    extern int s31_selftest_init(void);
    extern int rt_hw_pwm_init(void);
    extern int rt_hw_timer_init(void);
    extern int rt_hw_int_wdt_init(void);

    /* The interrupt watchdog, which the FreeRTOS path starts inside
     * esp_startup_start_app() and this port therefore does not get. See
     * port/esp32s31/boot_tasks.c for the full list of what that branch skips
     * and why the task watchdog is deliberately not among them. */
    rt_hw_int_wdt_init();

    rt_hw_pin_init();

    /* PWM and the general-purpose timers register nothing that blocks, so they
     * can come up before the scheduler like the pin device. Both are the only
     * reference that keeps their objects out of gc-sections: the msh commands
     * live in the same translation unit, and nothing else calls them. */
    rt_hw_pwm_init();
    rt_hw_timer_init();

    /* Compile-time tick consistency check; the same object carries the
     * `s31test` msh commands, which nothing else references. */
    s31_selftest_init();
#endif
}

/*
 * Give back the DRAM that IDF holds for the boot-time stack.
 *
 * soc_memory_regions (heap/port/<target>/memory_layout.c) fences off
 * [APP_USABLE_DRAM_END, SOC_DIRAM_DRAM_HIGH) as startup_stack=true, and
 * heap_caps_init deliberately leaves such a region unregistered with the
 * comment "Will be registered when OS scheduler starts". On the FreeRTOS path
 * that promise is kept by main_task -> reclaim_startup_stack_memory_for_heap()
 * -> heap_caps_enable_nonos_stack_heaps().
 *
 * startup.c:257 skips esp_startup_start_app() under CONFIG_IDF_RTOS_RTTHREAD, so
 * main_task never runs and nobody keeps it. On S31 that region is
 * 0x2f07afb0..0x2f080000, exactly 20 KB - measured, not estimated: `nm` shows
 * __heap_end__ at 0x2f045580 and the region ends at 0x2f080000, so the gap
 * between them is 20 KB larger than the registered heaps account for. With
 * Wi-Fi up (10 static RX buffers = 16 KB before anything else) that is not a
 * rounding error.
 *
 * Timing is the part that has to be right, and it is why this is an
 * INIT_APP_EXPORT rather than another line in rt_hw_board_init(): the region
 * contains SOC_ROM_STACK, and rt_hw_board_init() runs on that very stack. The
 * application-init stage runs from rt_components_init(), inside the main
 * thread, after the scheduler has switched to a stack the linker allocated - the
 * same point main_task occupies on the FreeRTOS path.
 */
static int rt_hw_reclaim_startup_stack(void)
{
    heap_caps_enable_nonos_stack_heaps();

    return 0;
}
INIT_APP_EXPORT(rt_hw_reclaim_startup_stack);

#if CONFIG_IDF_RTOS_RTTHREAD

/*
 * Start esp_timer before any SECONDARY-stage init function needs it.
 *
 * esp_timer splits in two. esp_timer_early_init() starts the counter as a
 * CORE-stage function, so esp_timer_get_time() works early. esp_timer_init()
 * is the other half - it creates the dispatch task and installs the alarm ISR -
 * and IDF reaches it from ESP_SYSTEM_INIT_FN(esp_timer_init_os, SECONDARY, ...,
 * 100), which esp_timer.c:660 compiles out under CONFIG_IDF_RTOS_RTTHREAD with
 * the comment "RT-Thread takes over the timers".
 *
 * That assumption does not hold here. RT-Thread has its own timers, but they are
 * not esp_timer, and IDF components still call esp_timer_create(): esp_coex does
 * it at SECONDARY priority 204 through ets_timer_setfn -> esp_timer_create, which
 * returns ESP_ERR_INVALID_STATE while s_timer_task is NULL (esp_timer.c:105).
 * ESP_ERROR_CHECK then aborts the boot at ets_timer_legacy.c:58.
 *
 * Calling it from rt_hw_board_init() instead - the first version of this fix -
 * is too late: board init runs from entry(), after do_secondary_init() has
 * already run every SECONDARY function. It only looked fine because the image
 * used to panic earlier, in do_core_init, before ever reaching coexist.
 *
 * Priority 99 precedes esp_timer_init_os (100, absent here), mbedtls PSA (104)
 * and coexist (204); lower numbers run first. esp_timer_init() is idempotent,
 * so a second call from anywhere is harmless.
 */
static esp_err_t rt_hw_esp_timer_init(void)
{
    extern int esp_timer_init(void);

    if (esp_timer_init() != 0)
    {
        rt_kprintf("board: esp_timer_init failed\n");
        return ESP_FAIL;
    }

    return ESP_OK;
}

ESP_SYSTEM_INIT_FN(rt_hw_esp_timer_init, SECONDARY, BIT(0), 99)
{
    return rt_hw_esp_timer_init();
}

#endif /* CONFIG_IDF_RTOS_RTTHREAD */

#if CONFIG_IDF_RTOS_RTTHREAD_FREERTOS_SHIM

/*
 * Start the tasks that were created before the scheduler existed.
 *
 * IDF's SECONDARY-stage init functions run before rtthread_startup() reaches
 * rt_system_scheduler_init(), yet several of them create tasks: esp_timer is the
 * first, and anything else that pulls in a component with a worker thread
 * follows. Creating is safe that early, starting is not - see
 * ft_task_start_pending().
 *
 * INIT_COMPONENT_EXPORT, not INIT_APP_EXPORT: this has to run before the
 * application stage so that nothing in the app stage finds a worker thread that
 * has not started. rt_components_init() runs from main_thread_entry, i.e. after
 * rt_system_scheduler_start(), which is exactly the window needed.
 */
static int rt_hw_start_pending_tasks(void)
{
    extern void ft_task_start_pending(void);

    ft_task_start_pending();

    return 0;
}
INIT_COMPONENT_EXPORT(rt_hw_start_pending_tasks);

#endif /* CONFIG_IDF_RTOS_RTTHREAD_FREERTOS_SHIM */
