/*
 * The boot-time work that IDF does on the FreeRTOS path and that this port has
 * to do instead, because startup.c:257 replaces esp_startup_start_app() with
 * `entry()` under CONFIG_IDF_RTOS_RTTHREAD.
 *
 * Nothing here is exotic. It is the list of things in startup.c /
 * app_startup.c that are skipped by that branch, checked one at a time rather
 * than assumed - "the image links and the shell answers" says nothing about
 * whether the interrupt watchdog exists.
 *
 * The five call sites the FreeRTOS path has and this one does not:
 *
 *   1. esp_int_wdt_init() + esp_int_wdt_cpu_init()   [app_startup.c:63, in
 *      esp_startup_start_app()]  - the interrupt watchdog. CONFIG_ESP_INT_WDT=y
 *      is set in this project, yet nm showed neither symbol in the image: the
 *      config option was simply inert.
 *   2. esp_crosscore_int_init()  [app_startup.c:73]  - inter-core interrupts.
 *      CONFIG_FREERTOS_UNICORE=y, so there is no other core to poke.
 *   3. xTaskCreatePinnedToCore(main_task)  [app_startup.c:78] - replaced by the
 *      RT-Thread main thread.
 *   4. reclaim_startup_stack_memory_for_heap()  [app_startup.c:180] - 20 KB of
 *      DRAM held back for the boot stack and never handed to the heap. Done from
 *      board.c, where it can run on a linker-allocated stack.
 *   5. esp_task_wdt_init()  [app_startup.c:197] - the task watchdog. Unlike 1,
 *      nothing in this image calls esp_task_wdt_add() (checked: zero call sites
 *      outside the component itself), so starting a watchdog with no subscribers
 *      would add a failure mode and catch nothing. Left out on purpose, and
 *      recorded here so the omission is a decision rather than an oversight.
 *
 * The interrupt watchdog is the opposite case: it fires when interrupts are
 * masked for too long, which needs no cooperation from any subscriber. Getting
 * stuck with interrupts off is exactly the failure a bring-up image wants to
 * report instead of hanging, so it is started below.
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
 * RT-Thread's tick path feeds it too - verified in port_systick.c:210-214.
 */
int rt_hw_int_wdt_init(void)
{
#if CONFIG_ESP_INT_WDT
    esp_int_wdt_init();
    esp_int_wdt_cpu_init();
#endif

    return 0;
}

#endif /* SOC_ESP32_S31 */
