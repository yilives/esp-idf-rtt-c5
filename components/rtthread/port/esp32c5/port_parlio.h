#ifndef PORT_PARLIO_H
#define PORT_PARLIO_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pads. IO1 and IO14 are taken by MCPWM/USB; IO0 is RMT. IO5 is PCNT's own test
 * pin, so it is not reused here. IO13 is free since PCNT moved off it (section
 * 17j). IO12 is the console's RX, so not that either. */
#define C5_PARLIO_CLK_PIN   13
#define C5_PARLIO_DATA_PIN  14

rt_err_t rt_hw_parlio_init(void);
rt_err_t rt_hw_parlio_set_pin(rt_base_t clk_pin, rt_base_t data_pin);

/* Clock `len` bytes out, 1 bit per clock, with the AHB GDMA feeding the TX FIFO. */
/* `out_bits` receives tx_st0.tx_fifo_rd_bit_cnt - the number of bits the unit
 * says it pulled out of the FIFO. That is the hardware's own statement of what it
 * consumed, and unlike a clock-edge count it does not depend on how fast the CPU
 * noticed the EOF. Pass NULL if not needed. */
rt_err_t rt_hw_parlio_tx(const rt_uint8_t *buf, rt_uint32_t len,
                         rt_uint32_t clk_hz, rt_uint32_t timeout_ms,
                         rt_uint32_t *out_bits);

/* Transmit and count the clocks with PCNT; the count must equal bytes * 8. */
int c5_parlio_selftest(rt_uint32_t len, rt_uint32_t clk_hz);

#ifdef __cplusplus
}
#endif

#endif /* PORT_PARLIO_H */

