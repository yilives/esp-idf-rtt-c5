#ifndef PORT_RMT_H
#define PORT_RMT_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One RMT group, 2 TX (ch0/ch1) + 2 RX (ch2/ch3) on this chip. */
#define C5_RMT_GROUP        0
#define C5_RMT_TX_CH        0

/* The RX channel index and the RX MEMORY channel are different numbers, and
 * confusing them is what broke the first version of this driver.
 *
 * `chmconf[]` (and every rmt_ll_rx_* accessor) is indexed by the RX channel's
 * position WITHIN the RX half of the group: 0 or 1 on this chip. The array has
 * exactly two elements, so passing 2 writes past the end of it - the register
 * writes land in whatever follows, the channel is never enabled, and the
 * receiver silently captures nothing.
 *
 * The symbol MEMORY, on the other hand, is addressed by the global channel
 * number, which is this index + 2 (matching
 * rmt_ll_rx_get_memory_writer_offset(), which subtracts exactly `(channel + 2)
 * * 48`). Hence two defines. */
#define C5_RMT_RX_CH        0       /* index for rmt_ll_rx_* / chmconf[] */
#define C5_RMT_RX_MEM_CH    2       /* index into RMTMEM.channels[] */

/* Free pin: console IO11/IO12, PWM IO3/IO5/IO7, I2C IO8/IO9, SPI IO2/IO6/IO7/IO10,
 * CAN IO4, in-package flash IO15-IO22. IO0 is unused on this board. */
#define C5_RMT_TEST_PIN     0

rt_err_t rt_hw_rmt_init(void);
rt_err_t rt_hw_rmt_set_pin(rt_base_t pin);
rt_err_t rt_hw_rmt_tx(rt_uint32_t symbols, rt_uint32_t timeout_ms);
rt_uint32_t rt_hw_rmt_tick_ns(void);

/* Transmit a square wave and measure it with the RX channel. */
int c5_rmt_selftest(rt_uint32_t half_us);

#ifdef __cplusplus
}
#endif

#endif /* PORT_RMT_H */

