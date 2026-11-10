#ifndef PORT_PARLIO_H
#define PORT_PARLIO_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pads. IO1 and IO14 are taken by MCPWM/USB; IO0 is RMT. IO5 is PCNT's own test
 * pin, so it is not reused here. IO13 is free since PCNT moved off it (section
 * 17j). IO12 is the console's RX, so not that either. */
/* PARLIO gets its OWN GDMA channel.
 *
 * It shared channel 0 with the M2M self test, and no amount of cleanup on either
 * side made them independent: measured, `c5dma` followed by `c5parlio` in one
 * boot transferred 88 of 128 bits with `dma_txint=0x1` (TX_DONE) while PARLIO's
 * own status stayed empty, and `c5parlio` alone (or twice in a row) passed. Two
 * drivers taking turns on one channel have to agree on every field; the C5 has
 * three channel pairs (`GDMA_LL_AHB_PAIRS_PER_GROUP` is 3), so they do not have
 * to.
 *
 * Channel 1: its `peri_out_sel` is written explicitly rather than left as the
 * hardware-assigned M2M id, so the value does not depend on the channel number. */
#define C5_PARLIO_GDMA_TX_CH    1

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

