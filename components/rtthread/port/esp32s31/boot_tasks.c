/*
 * The boot-time work that IDF does on the FreeRTOS path and that this port has
 * to do instead, because startup.c replaces esp_startup_start_app() with `entry()`
 * under CONFIG_IDF_RTOS_RTTHREAD.
 *
 * Every claim below was re-checked against the source and the ELF in W70, not
 * carried over from the first draft. Two of the five were stale.
 *
 * The five call sites the FreeRTOS path has and this one does not:
 *
 *   1. esp_int_wdt_init() + esp_int_wdt_cpu_init()   [app_startup.c, in
 *      esp_startup_start_app()]  - the interrupt watchdog. CONFIG_ESP_INT_WDT=y is
 *      set in this project, yet nm showed neither symbol in the image before this
 *      file existed: the config option was inert. Both are present now
 *      (0x400982b2 / 0x4009836e), so the option is live. Done below.
 *
 *   2. esp_crosscore_int_init()  [app_startup.c]  - IDF's inter-core interrupt.
 *      Not started, and RT-Thread does not need it: the MP scheduler calls
 *      rt_hw_ipi_send(), which this port implements directly on the CLIC line IDF
 *      reserves for crosscore traffic (port/esp32s31/smp_ipi.c). Verified in nm:
 *      rt_hw_ipi_send is in the image, esp_crosscore_int_* is not.
 *
 *      The reason recorded here used to be "CONFIG_FREERTOS_UNICORE=y, so there is
 *      no other core to poke". That was true when P1 ran on one core; it is not any
 *      more. RT_USING_SMP with RT_CPUS_NR=2 has been booting since W56 and CPU1
 *      executes its idle thread, so the mechanism matters even though the conclusion
 *      (no need for IDF's version) still holds.
 *
 *   3. xTaskCreatePinnedToCore(main_task)  [app_startup.c] - replaced by the
 *      RT-Thread main thread.
 *
 *   4. reclaim_startup_stack_memory_for_heap()  [app_startup.c] - the region the
 *      heap registers with startup_stack=true and refuses to hand out until the OS
 *      scheduler runs. On S31 that is APP_USABLE_DRAM_END..SOC_DIRAM_DRAM_HIGH
 *      = (SOC_ROM_STACK_START - SOC_ROM_STACK_SIZE)..0x2f080000
 *      = 0x2f07afb0..0x2f080000 = 20560 B. Done from board.c, where it can run on a
 *      linker-allocated stack. The numbers here are recomputed from soc.h and
 *      memory_layout.c rather than remembered.
 *
 *   5. esp_task_wdt_init()  [app_startup.c] - the task watchdog. Nothing in this
 *      image calls esp_task_wdt_add(): the five hits in the tree are this comment,
 *      four test_apps, and nothing in components/. Starting a watchdog with no
 *      subscribers would add a failure mode and catch nothing. Left out on purpose,
 *      and recorded here so the omission is a decision rather than an oversight.
 *
 * The interrupt watchdog is the opposite case: it fires when interrupts are masked
 * for too long, which needs no cooperation from any subscriber. Getting stuck with
 * interrupts off is exactly the failure a bring-up image wants to report instead of
 * hanging, so it is started below.
 */

#include <rtthread.h>

#include "sdkconfig.h"
#include "esp_private/esp_int_wdt.h"

#if defined(SOC_ESP32_S31)

/*
 * Called from rt_hw_board_init(), before the scheduler.
 *
 * Safe at that point: esp_int_wdt_init() only touches the TIMG1 watchdog
 * registers, and esp_int_wdt_cpu_init() registers a tick hook plus routes
 * ETS_INT_WDT_INUM. The hook is in freertos_hooks.c, which port_systick.c calls
 * from xPortSysTickHandler() *before* the CONFIG_IDF_RTOS_RTTHREAD branch, so
 * RT-Thread's tick path feeds it too - verified in port_systick.c:212.
 */
rt_err_t rt_hw_int_wdt_init(void)
{
#if CONFIG_ESP_INT_WDT
    esp_int_wdt_init();
    esp_int_wdt_cpu_init();
#endif

    return RT_EOK;
}

#endif /* SOC_ESP32_S31 */
