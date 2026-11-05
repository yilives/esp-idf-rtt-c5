/*
 * TWAI (CAN / CAN-FD) on the S31: classic CAN frames, as a plain C API.
 *
 * RT-Thread does have a `rt_can` device class and this one will get registered
 * into it - but only after the hardware path is proven, so that a failing rt_can
 * test cannot be confused with a failing controller.
 *
 * ---------------------------------------------------------------------------
 * Verification without a second node and without a transceiver.
 *
 * A CAN transmitter normally needs another node to acknowledge each frame; with
 * nobody to ACK it retries forever and any test can only report "timeout". The
 * S31 controller has a self-test mode (`stm`) that loops TX back to RX inside the
 * chip and does not wait for an acknowledgement, which is what closes the loop
 * on a bare board.
 *
 * `s31twai` therefore transmits a frame with a known ID and payload and reads it
 * back out of the RX FIFO, comparing ID, length and every byte - at several bit
 * rates, because a wrong BRP shows up as one rate failing while its neighbours
 * work.
 *
 * NOT verified, and stated as such: the analog layer. No transceiver is fitted,
 * so there is no differential pair, no termination, no arbitration against a
 * real second node and no error-frame behaviour. What is verified is the
 * controller: timing registers, frame formatting, TX buffer handoff, RX FIFO,
 * and the data path in both directions.
 *
 * ---------------------------------------------------------------------------
 * S31 specifics:
 *
 *  - IDF's C API says TWAI, the hardware block and its register headers say
 *    TWAIFD. This part has a CAN-FD controller and only the FD register set
 *    exists for it - there is no classic `twai_ll.h` frame API on the S31.
 *    Classic frames are FD buffers with `fdf = 0`.
 *
 *  - Two controllers: TWAIFD0 at 0x20394000, TWAIFD1 at 0x20395000.
 *
 *  - The controller memory has its own low-power domain
 *    (HP_SYSTEM.sys_canN_mem_lp_ctrl), the same shape as the RMT symbol memory.
 *    That cost three rounds on the RMT, so it is handled up front here.
 *
 *  - TX buffers are addressed cells (txt_mem_cell[]) armed with a one-hot
 *    command bit - a different model from the RMT's streaming FIFO, with no
 *    equivalent pointer problem.
 */

#ifndef __PORT_TWAI_H__
#define __PORT_TWAI_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Clock, memory power and self-test mode. Call once. */
rt_err_t rt_hw_twai_init(void);

/**
 * Set the bus bitrate (classic CAN).
 *
 * Uses 16 quanta per bit with a 75 % sample point; if the pre-divider would
 * overflow it falls back to 8 quanta.
 */
rt_err_t rt_hw_twai_set_bitrate(rt_uint32_t bitrate);

/** Route the controller to a pair of pads. */
rt_err_t rt_hw_twai_set_pins(rt_base_t tx_pin, rt_base_t rx_pin);

/** Queue one standard-ID classic frame (max 8 bytes). */
rt_err_t rt_hw_twai_send(rt_uint32_t id, const rt_uint8_t *data, rt_uint32_t len,
                         rt_uint32_t timeout_ms);

/** Read one frame from the RX buffer, or time out. */
rt_err_t rt_hw_twai_recv(rt_uint32_t *id, rt_uint8_t *data, rt_uint32_t *len,
                         rt_uint32_t timeout_ms);

/* Debug accessors, used by the command's failure path only. */
rt_uint32_t s31_twai_dbg_tx_status(void);
rt_uint32_t s31_twai_dbg_rx_count(void);
int s31_twai_dbg_rx_empty(void);
rt_uint32_t s31_twai_dbg_tec(void);
rt_uint32_t s31_twai_dbg_rec(void);
int s31_twai_dbg_state(void);
rt_uint32_t s31_twai_dbg_intr(void);
rt_uint32_t s31_twai_dbg_mode(void);

/**
 * Referenced from board.c so --gc-sections keeps the msh command.
 *
 * Renamed from s31_twai_cmd_init(): it is a link anchor, not an initialiser.
 */
void s31_twai_cmd_link_anchor(void);

rt_err_t rt_hw_twai_start(void);
void rt_hw_twai_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_TWAI_H__ */
