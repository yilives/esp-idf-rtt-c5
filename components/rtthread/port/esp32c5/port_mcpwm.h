#ifndef PORT_MCPWM_H
#define PORT_MCPWM_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One group (MCPWM0), 3 timers, 3 operators, 2 generators per operator. */
#define C5_MCPWM_GROUP      0
#define C5_MCPWM_TIMER      0
#define C5_MCPWM_OPER       0

/* Free pins: console IO11/IO12, PWM(LEDC) IO3/IO5/IO7, I2C IO8/IO9,
 * SPI IO2/IO6/IO7/IO10, CAN IO4, RMT IO0, PCNT IO13, flash IO15-IO22.
 * IO1 and IO14 are unused. */
#define C5_MCPWM_TEST_PIN_A 1
#/* Output B is programmed (complementary pair, dead time) but NOT routed to a pad.
 *
 * -1 means "no pad", and rt_hw_mcpwm_set_pin() already handles it. The reason is
 * that every usable pad on this board is taken and IO14 - the obvious choice - is
 * the native USB D+ pad. Routing B cost nothing to verify (only A is measured) and
 * would have put a self test on a hardware pin for no gain. */
#define C5_MCPWM_TEST_PIN_B (-1)

/* Dead time in timer ticks. At 48 MHz with prescaler 1 that is ~42 ns, well
 * below anything a real half-bridge needs - the value here is only meant to prove
 * the programming path runs, not to protect a transistor. */
#define C5_MCPWM_DEAD_TICKS 2

rt_err_t rt_hw_mcpwm_init(void);
rt_err_t rt_hw_mcpwm_setup(rt_uint32_t freq_hz, rt_uint32_t duty_permille,
                           rt_uint32_t dead_ticks);
void     rt_hw_mcpwm_set_duty(rt_uint32_t duty_permille);
void     rt_hw_mcpwm_start(void);
void     rt_hw_mcpwm_stop(void);
rt_err_t rt_hw_mcpwm_set_pin(rt_base_t pin_a, rt_base_t pin_b);

/* Frequency by PCNT, duty by mcycle polling. Both measured at the pad. */
int c5_mcpwm_selftest(rt_uint32_t freq_hz, rt_uint32_t duty_permille);

#ifdef __cplusplus
}
#endif

#endif /* PORT_MCPWM_H */

