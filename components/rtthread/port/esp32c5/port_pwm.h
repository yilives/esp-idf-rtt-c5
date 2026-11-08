#ifndef PORT_PWM_H
#define PORT_PWM_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Six channels, four timers, one speed mode - from soc_caps.h, not guessed. */
#define C5_PWM_CHANNELS     6
#define C5_PWM_DEV_NAME     "pwm0"

/* Free on this board: the console is UART0 on its default pads and the bring-up
 * image's only other pin user is the GPIO self test, which uses IO7. Not a
 * strapping pin and not a flash pad, so driving it cannot brick a boot. */
#define C5_PWM_TEST_PIN     24

rt_err_t rt_hw_pwm_init(void);

/* Route a channel to a GPIO and program period/pulse. Exposed because the
 * self test uses it and because an application that wants a specific pin needs
 * the same call. */
rt_err_t c5_pwm_set_pin(rt_uint32_t channel, rt_base_t pin);

/* Drive and read back; returns 0 on PASS. */
int c5_pwm_selftest(int pin, int hz, int duty_pct);

#ifdef __cplusplus
}
#endif

#endif /* PORT_PWM_H */

