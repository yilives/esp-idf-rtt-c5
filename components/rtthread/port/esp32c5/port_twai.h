#ifndef PORT_TWAI_H
#define PORT_TWAI_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TWAI0. Two controllers exist; only the first is brought up. */
#define C5_TWAI_ID          0

/* Free pins on this board: console IO11/IO12, PWM IO3/IO5/IO7, I2C IO8/IO9,
 * SPI IO2/IO6/IO7/IO10, in-package flash IO15-IO22. IO4 and IO13 are unused. */
/* TX and RX on the SAME pad. This is a legal configuration on this part because
 * TWAI0_TX_IDX and TWAI0_RX_IDX are the same matrix index (66): the pad's own
 * input sees what its own output drives, so a frame goes out and comes straight
 * back with no jumper and without relying on the controller's internal loopback
 * bit. The S31 file documents the same trick as its way of separating "ilbp is
 * not taking effect" from "the frame never reaches the pin". */
/* MOVED OFF IO4 because the LP UART now owns it.
 *
 * The LP UART's pads are fixed by the LP IO MUX (this part does not define
 * SOC_LP_GPIO_MATRIX_SUPPORTED, so `void` cannot be routed elsewhere): TXD is
 * IO5 and RXD is IO4, full stop. CAN goes through the GPIO matrix and can sit on
 * any pad, so CAN is the one that moves.
 *
 * The conflict was real, not theoretical - measured before the move:
 *
 *     c5can     -> PASS
 *     c5lpuart  -> PASS
 *     c5can     -> FAIL, write through the device class
 *
 * `c5pins` reported PASS throughout, because its table still listed IO4 as CAN
 * and did not know the LP UART existed. A pin table that is not updated when a
 * driver is added is worse than no table: it certifies a conflict. */
#define C5_TWAI_TX_PIN      26
#define C5_TWAI_RX_PIN      26

rt_err_t rt_hw_twai_init(void);
rt_err_t rt_hw_twai_set_bitrate(rt_uint32_t bitrate);
rt_err_t rt_hw_twai_set_pins(rt_base_t tx_pin, rt_base_t rx_pin);
rt_err_t rt_hw_twai_start(void);
void     rt_hw_twai_stop(void);
/* Send a frame and report WHICH hardware TX buffer it went into.
 *
 * The device class keeps its own box numbering, unrelated to this controller's
 * buffer indices, and `rt_hw_can_isr()` is addressed by the FORMER. Without the
 * hardware index coming back out, the completion cannot be attributed to the
 * right box - and attributing it to box 0 is wrong the moment the driver stops
 * always using buffer 0, which it does as soon as buffer 0 is left in TX_OK. */
rt_err_t rt_hw_twai_send_ex(rt_uint32_t id, const rt_uint8_t *data,
                            rt_uint32_t len, rt_uint32_t timeout_ms,
                            rt_uint32_t *out_buf_idx);

rt_err_t rt_hw_twai_send(rt_uint32_t id, const rt_uint8_t *data, rt_uint32_t len,
                         rt_uint32_t timeout_ms);
/* Return every TX buffer that has finished back to the Empty state and report
 * which ones completed.
 *
 * This is the missing half of the transmit path. A buffer that finished a frame
 * sits in TX_OK (state 4) or Failed (state 6) and NEITHER accepts a new frame -
 * only Empty (8) does. Nothing returned them, so the transmit path consumed one
 * buffer per frame and then found none.
 *
 * The return value is a bitmask of the buffers that were reclaimed (bit 0 =
 * buffer 0), so the caller can report completion for the buffer that actually
 * finished instead of assuming it was buffer 0. */
rt_uint32_t rt_hw_twai_reclaim_tx_buffers(void);

/* Total TX buffers this controller has. */
rt_uint32_t rt_hw_twai_tx_buffer_count(void);

rt_err_t rt_hw_twai_recv(rt_uint32_t *id, rt_uint8_t *data, rt_uint32_t *len,
                         rt_uint32_t timeout_ms);

/* Round-trip a frame at 125k/250k/500k/1M in internal loopback. */
int c5_twai_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_TWAI_H */

