#ifndef PORT_TSENS_H
#define PORT_TSENS_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returned when the sensor cannot produce a value. Chosen outside any plausible
 * die temperature so a caller can tell it from a real reading. */
#define C5_TSENS_INVALID    (-1000)

rt_err_t rt_hw_tsens_init(void);
void     rt_hw_tsens_deinit(void);

/* Degrees Celsius, uncalibrated. */
int      rt_hw_tsens_read(void);

/* The raw SAR reading behind the last conversion, for diagnostics. */
int      rt_hw_tsens_raw(void);

/* The DAC offset of the selected range, in DAC_FACTOR units. */
rt_int8_t rt_hw_tsens_dac_offset(void);

/* Plausibility + noise + self-heating. */
int c5_tsens_selftest(rt_uint32_t samples);

#ifdef __cplusplus
}
#endif

#endif /* PORT_TSENS_H */
