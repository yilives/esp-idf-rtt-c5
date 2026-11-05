/*
 * LEDC PWM device, exposed as the standard RT-Thread PWM class.
 *
 *   rt_device_find("pwm0")  ->  rt_pwm_set(dev, ch, period_ns, pulse_ns)
 *                               rt_pwm_enable(dev, ch)
 *
 * Before a channel can drive anything its pad has to be assigned, because the
 * RT-Thread PWM interface carries no pin:
 *
 *   s31_pwm_set_pin(ch, gpio);
 *
 * Channels share timers four ways: channel c uses timer c % 4, so setting the
 * period on channel 0 changes the period of channels 4 and 8 as well. With only
 * four LEDC timers and eight channels there is no way around that; if the
 * periods must differ, use channels that land on different timers.
 */

#ifndef __PORT_PWM_H__
#define __PORT_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif


#include <rtthread.h>

#define S31_PWM_DEV_NAME    "pwm0"
#define S31_PWM_CHANNELS    SOC_LEDC_CHANNEL_NUM

/** Register "pwm0". Called from rt_hw_board_init(). */
rt_err_t rt_hw_pwm_init(void);

/**
 * Route a channel to a GPIO pad and put the pad in output mode.
 *
 * @param channel 0..S31_PWM_CHANNELS-1
 * @param pin     GPIO number
 * @return RT_EOK, or -RT_EINVAL when either argument is out of range
 */
rt_err_t s31_pwm_set_pin(rt_uint32_t channel, rt_base_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_PWM_H__ */
