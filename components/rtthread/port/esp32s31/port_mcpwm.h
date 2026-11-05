/*
 * MCPWM on the S31: one complementary pair with dead time.
 *
 * RT-Thread has no motor-control device class (no rt_mcpwm_* API to satisfy), so
 * this is a plain C API plus an msh command. Same reasoning as port_pcnt.h: an
 * rt_device wrapper with one caller adds indirection without adding a contract.
 *
 * Self-verification without a scope
 * ---------------------------------
 * The A output pad is also readable as a GPIO, so `s31mcpwm` closes the loop
 * inside the chip:
 *
 *   - PCNT counts rising edges on A over 50 ms        -> frequency
 *   - mcycle polling of A measures high vs period     -> duty
 *
 * What is NOT verified and is stated as such: the analog shape of the output
 * (rise time, overshoot) and the dead time's actual nanoseconds. Dead time is
 * programmed and read back from its register, but not measured - that needs a
 * scope.
 *
 * Clock: XTAL (40 MHz), not PLL. The PWM frequency then does not move when DFS
 * changes the CPU clock, which is the one property a motor control signal must
 * have.
 */

#ifndef __PORT_MCPWM_H__
#define __PORT_MCPWM_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Enable the group clock and select XTAL as the source. Call once. */
rt_err_t rt_hw_mcpwm_init(void);

/**
 * Program operator 0 / timer 0 as a complementary pair.
 *
 * Generator A: high from zero to compare, low after - i.e. a high pulse whose
 * width is `duty_permille` of the period. Generator B is its complement, with
 * `dead_ticks` of dead time inserted on both edges.
 *
 * @param freq_hz      50 .. 100000
 * @param duty_permille 0 .. 1000
 * @param dead_ticks    dead time in timer ticks (0 disables the delays)
 */
rt_err_t rt_hw_mcpwm_setup(rt_uint32_t freq_hz, rt_uint32_t duty_permille,
                           rt_uint32_t dead_ticks);

/** Change the duty without touching the frequency. */
void rt_hw_mcpwm_set_duty(rt_uint32_t duty_permille);

/** Route output A and B to pads. Pass -1 to leave a pad unwired. */
rt_err_t rt_hw_mcpwm_set_pin(rt_base_t pin_a, rt_base_t pin_b);

void rt_hw_mcpwm_start(void);
void rt_hw_mcpwm_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_MCPWM_H__ */
