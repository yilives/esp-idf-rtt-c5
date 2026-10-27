#include <rtthread.h>
#include "sdkconfig.h"
#include "port_systick.h"
#include "riscv/interrupt.h"

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
#if CONFIG_IDF_RTOS_RTTHREAD_FREERTOS_SHIM
    {
        extern int esp_timer_init(void);

        if (esp_timer_init() != 0)
        {
            rt_kprintf("board: esp_timer_init failed\n");
        }
    }
#endif

    /* Register the pin device before the scheduler runs so anything that uses
     * rt_pin_*() during init finds it. Also the reference that keeps the GPIO
     * driver object from being dropped out of the component archive:
     * rt_device_pin_register() lives in the drivers archive, not here. */
    extern int rt_hw_pin_init(void);
    extern int s31_selftest_init(void);
    extern int rt_hw_pwm_init(void);
    extern int rt_hw_timer_init(void);

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
