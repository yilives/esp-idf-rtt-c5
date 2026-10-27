#include <rtthread.h>
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
