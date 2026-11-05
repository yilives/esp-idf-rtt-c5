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
#define C5_TWAI_TX_PIN      4
#define C5_TWAI_RX_PIN      4

rt_err_t rt_hw_twai_init(void);
rt_err_t rt_hw_twai_set_bitrate(rt_uint32_t bitrate);
rt_err_t rt_hw_twai_set_pins(rt_base_t tx_pin, rt_base_t rx_pin);
rt_err_t rt_hw_twai_start(void);
void     rt_hw_twai_stop(void);
rt_err_t rt_hw_twai_send(rt_uint32_t id, const rt_uint8_t *data, rt_uint32_t len,
                         rt_uint32_t timeout_ms);
rt_err_t rt_hw_twai_recv(rt_uint32_t *id, rt_uint8_t *data, rt_uint32_t *len,
                         rt_uint32_t timeout_ms);

/* Round-trip a frame at 125k/250k/500k/1M in internal loopback. */
int c5_twai_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_TWAI_H */

