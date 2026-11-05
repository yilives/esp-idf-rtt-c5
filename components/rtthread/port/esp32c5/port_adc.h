#ifndef PORT_ADC_H
#define PORT_ADC_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One SAR ADC unit, six channels, 12-bit - from soc_caps.h, not guessed. */
#define C5_ADC_DEV_NAME     "adc1"
#define C5_ADC_CHANNELS     6

/* ADC1 channel N is GPIO(N+1) on this chip (soc/adc_channel.h). GPIO3 doubles as
 * the PWM self-test pin and GPIO1/GPIO2 are the LP_UART pads. */
#define C5_ADC_TEST_CHANNEL 0

rt_err_t rt_hw_adc_init(void);

/* Read every channel; returns 0 on PASS. */
int c5_adc_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_ADC_H */
