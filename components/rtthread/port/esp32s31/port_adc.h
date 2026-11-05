/*
 * SAR ADC on the S31, registered as `adc1` / `adc2`.
 *
 * 17-bit raw results, 8 channels per unit, oneshot only.
 *
 * Two things this driver deliberately does not do, both because the hardware on
 * this SoC does not allow them honestly:
 *
 *   1. No millivolt conversion. `get_vref()` returns -1 ("unknown").
 *
 *      The S31 has neither SOC_ADC_CALIBRATION_V1_SUPPORTED nor an RTC
 *      controller, so the IDF calibration component cannot produce raw-to-mV
 *      coefficients for it. Any constant "typical Vref" written here would look
 *      authoritative and be wrong by several percent on a given board, and
 *      every downstream reading would inherit the error without any indication.
 *      A raw code is a measurement; a guessed voltage is not.
 *
 *      If calibrated voltages are needed, the calibration data has to be read
 *      out of the chip's eFuse/analog path first - that is real work, not a
 *      default value.
 *
 *   2. No continuous/DMA mode. Oneshot is what the RT-Thread ADC framework
 *      models, and it is the only mode that can be verified on this bench.
 *
 * Pin map (fixed silicon, soc/adc_channel.h - NOT a board choice):
 *
 *   adc1 ch0..7 -> GPIO42..49
 *   adc2 ch0..7 -> GPIO50..57
 *
 * IMPORTANT COLLISION: GPIO50/51 are the I2C pins on this board (I2C_SCL /
 * I2C_SDA, see port_board.h). ADC2 channel 0 and 1 share those pads, so ADC2
 * and i2c0 cannot be used at the same time. `s31adc 2 0` refuses unless
 * `force` is given, and says why.
 */

#ifndef __PORT_ADC_H__
#define __PORT_ADC_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Bring up both SAR units and register `adc1` / `adc2`. */
rt_err_t rt_hw_adc_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_ADC_H__ */
