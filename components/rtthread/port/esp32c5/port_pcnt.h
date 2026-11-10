#ifndef PORT_PCNT_H
#define PORT_PCNT_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One group (PCNT), four units, two channels each. */
#define C5_PCNT_UNITS   4

/* Free pin: console IO11/IO12, PWM IO3/IO5/IO7, I2C IO8/IO9, SPI IO2/IO6/IO7/IO10,
 * CAN IO4, RMT IO0, in-package flash IO15-IO22. */
#/* IO5, not IO13.
 *
 * IO13 and IO14 are the C5's native USB D-/D+ pads (USB_INT_PHY0_DM/DP_GPIO_NUM
 * in io_mux_reg.h). They work as ordinary GPIO while nothing enumerates USB, which
 * is why the self test passed on IO13 - but a test pad that collides with a
 * hardware function is a trap for whoever next enables the USB serial/JTAG
 * console. `c5pins` reports the clash; this is the fix it asked for. */
/* MOVED OFF IO5 for the same reason as CAN: IO5 is the LP UART's TXD and that
 * assignment cannot move. PCNT counts through the GPIO matrix, so it can. */
#define C5_PCNT_TEST_PIN    27

rt_err_t rt_hw_pcnt_init(void);
rt_err_t rt_hw_pcnt_setup(int unit, rt_base_t pin);
int      rt_hw_pcnt_get(int unit);
void     rt_hw_pcnt_clear(int unit);
void     rt_hw_pcnt_stop(int unit);

/* Drive `edges` rising edges on `pin` from software and compare with the counter. */
int c5_pcnt_selftest(rt_base_t pin, rt_uint32_t edges);

#ifdef __cplusplus
}
#endif

#endif /* PORT_PCNT_H */


