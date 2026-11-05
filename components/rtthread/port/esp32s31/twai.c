/*
 * ESP32-S31 TWAI (CAN / CAN-FD): classic CAN frames, as a plain C API.
 *
 * RT-Thread has a `rt_can` device class, and unlike PCNT/MCPWM/RMT this one is
 * worth wiring up - but only after the hardware path is proven. This file
 * proves the path first; the rt_can registration is a separate step.
 *
 * ---------------------------------------------------------------------------
 * How this is verified with no second node and no transceiver.
 *
 * A CAN controller normally needs another node to acknowledge every frame; with
 * nobody to ACK, a transmitter retries forever and the test can only report
 * "timed out". The S31 TWAI-FD controller has a self-test mode (`stm`) that
 * routes TX back to RX inside the chip and does not wait for an ACK, which
 * closes the loop on a bare board.
 *
 * So `s31twai` does:
 *   1. puts the controller in self-test mode
 *   2. transmits a frame with a known ID and payload
 *   3. reads the frame back out of the RX FIFO
 *   4. compares id, length and every data byte
 *
 * It also runs the same test at several bit rates, because a wrong BRP shows up
 * as a bitrate that does not work while its neighbour does.
 *
 * What is NOT verified and is stated as much: the analog layer. No transceiver
 * is fitted, so there is no differential pair, no bus termination, no
 * arbitration against a real second node, and no error-frame behaviour. What is
 * verified is the controller: timing registers, frame formatting, TX buffer
 * handoff, RX FIFO, and the data path in both directions.
 *
 * ---------------------------------------------------------------------------
 * S31 specifics:
 *
 *  - The peripheral is called TWAI in IDF's C API but the hardware block and its
 *    register headers are TWAIFD: this part has a CAN-FD controller, and only
 *    the FD-capable register set exists for it. There is no `twai_ll.h` frame
 *    API on the S31 - `twaifd_ll.h` is the one to use, and the classic-CAN
 *    functions live there with an `fdf = 0` header.
 *
 *  - Two controllers, TWAIFD0 at 0x20394000 and TWAIFD1 at 0x20395000.
 *
 *  - The controller memory has its own low-power domain
 *    (HP_SYSTEM.sys_canN_mem_lp_ctrl), exactly like the RMT symbol memory. That
 *    lesson cost three rounds on the RMT, so it is handled up front here.
 *
 *  - Writes to a TX buffer go through the APB window at txt_mem_cell[]; the frame
 *    is then armed with a one-hot command bit. That is a different model from
 *    the RMT's streaming FIFO and does not have the same pointer problem.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/twaifd_struct.h"
#include "hal/gpio_ll.h"
#include "hal/twai_ll.h"
#include "hal/twaifd_ll.h"
#include "hal/twai_types.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#include "port_twai.h"
#include "port_board.h"

#if defined(SOC_TWAI_SUPPORTED)

#define S31_TWAI_ID         0

/* 40 MHz XTAL / brp / quanta-per-bit = bitrate */
#define S31_TWAI_SRC_HZ     (CONFIG_XTAL_FREQ * 1000000u)

static twaifd_dev_t *s_twai;

/* Number of TX buffers the controller reports (4 on this part). */
static rt_uint32_t s_twai_buffer_count = 1u;

/*
 * "Empty" is state code 8, not 0.
 *
 * The 4-bit TX buffer state field resets to 8, so the natural-looking
 * `status == 0` test never matches and every send times out. Measured on the
 * first run: all four bit rates reported "TX buffer never became free".
 */
#define S31_TWAI_TXB_EMPTY   8u

static inline rt_bool_t s31_twai_buffer_free(rt_uint32_t idx)
{
    return twaifd_ll_get_tx_buffer_status(s_twai, (rt_uint8_t)idx) ==
           S31_TWAI_TXB_EMPTY;
}

static inline rt_uint32_t s31_twai_cycles(void)
{
    rt_uint32_t v;

    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));

    return v;
}

/*
 * Pick the timing for a bitrate.
 *
 * The bus is divided into `quanta` ticks of 1/(src/brp) each, and a bit is
 * 1 sync + prop_seg + tseg_1 + tseg_2 quanta. Sixteen quanta per bit is the
 * usual choice: enough resolution to place the sample point, few enough that brp
 * stays small at high rates.
 */
static rt_err_t s31_twai_timing(rt_uint32_t bitrate,
                                twai_timing_advanced_config_t *t)
{
    rt_uint32_t quanta = 0;
    rt_uint32_t brp = 0;
    rt_uint32_t q;

    if (bitrate == 0 || bitrate > 1000000u)
    {
        return -RT_EINVAL;
    }

    /*
     * Search for an EXACT divider instead of truncating one.
     *
     * brp and quanta are integers whose product must equal src / bitrate. Asking
     * for 16 quanta at 1 Mbit/s gives 40e6 / (1e6 * 16) = 2.5, which truncates
     * to 2 - and the bus then runs at 1.25 Mbit/s, 25 % fast. That is not a
     * rounding detail: it is the difference between a working bus and one that
     * fills its error counters.
     *
     * Measured on the previous version: 1 Mbit/s alone accumulated tec = 160
     * while the three lower rates stayed at 0.
     *
     * The search tries 8..25 quanta (CAN allows 8..25) and takes the exact
     * solution closest to 16, which keeps both the sample-point resolution and
     * brp reasonably small.
     */
    for (q = 8u; q <= 25u; q++)
    {
        rt_uint32_t total = S31_TWAI_SRC_HZ / bitrate;

        if (total % q != 0u)
        {
            continue;
        }

        if ((total / q) == 0u || (total / q) > TWAI_LL_BRP_MAX)
        {
            continue;
        }

        if (quanta == 0u || (q >= 16u && q < quanta) || (quanta < 16u && q > quanta))
        {
            quanta = q;
            brp = total / q;
        }
    }

    if (quanta == 0u)
    {
        /* No exact solution: fall back to the old truncating behaviour, but say
         * so rather than pretending the rate is exact. */
        return -RT_EINVAL;
    }

    /*
     * A bit is: 1 sync quanta + prop_seg + tseg_1 + tseg_2.
     *
     * The sync segment is mandatory and is NOT programmable, so the three
     * programmed segments must add up to quanta - 1. The first version split all
     * 16 quanta across the three, making every bit one quanta short: at 1 Mbit/s
     * that ran the bus at 1.33 Mbit/s instead of 1.0, which is why only the
     * highest rate produced error-counter growth.
     *
     * Sample point at 75 % of the bit: prop 25 %, tseg1 50 %, tseg2 25 %, all of
     * (quanta - 1).
     */
    {
        rt_uint32_t prog = quanta - 1u;

        t->brp      = brp;
        t->prop_seg = (rt_uint8_t)((prog / 4u) ? (prog / 4u) - 1u : 0u);
        t->tseg_1   = (rt_uint8_t)((prog / 2u) ? (prog / 2u) - 1u : 0u);
        t->tseg_2   = (rt_uint8_t)(prog - (prog / 4u) - (prog / 2u));
        if (t->tseg_2 < 1u)
        {
            t->tseg_2 = 1u;
        }
        t->sjw      = (prog / 8u) > 4u ? 4u : (rt_uint8_t)(prog / 8u);
        if (t->sjw == 0)
        {
            t->sjw = 1;
        }
    }
    t->ssp_offset = 0;

    return RT_EOK;
}

rt_err_t rt_hw_twai_init(void)
{
    twai_timing_advanced_config_t timing;

    PERIPH_RCC_ATOMIC()
    {
        twai_ll_enable_bus_clock(S31_TWAI_ID, true);
        twai_ll_reset_register(S31_TWAI_ID);
        twai_ll_enable_clock(S31_TWAI_ID, true);
    }

    /*
     * Power the controller memory. Its low-power domain defaults to off, the
     * same shape as the RMT symbol memory - and on the RMT that cost three
     * rounds of "the write never landed" before it was found. Handled first
     * here.
     */
    twai_ll_mem_force_power_on(S31_TWAI_ID);

    twai_ll_set_clock_source(S31_TWAI_ID, TWAI_CLK_SRC_XTAL);

    s_twai = &TWAIFD0;

    twaifd_ll_reset(s_twai);

    s_twai_buffer_count = twaifd_ll_get_tx_buffer_total(s_twai);
    if (s_twai_buffer_count == 0u || s_twai_buffer_count > 8u)
    {
        s_twai_buffer_count = 1u;
    }

    /* A mode change requires the controller to be disabled. */
    twaifd_ll_enable_hw(s_twai, false);
    /*
     * Self-test AND internal loopback. Both flags, and only setting the first
     * produces a controller that transmits happily and receives nothing:
     *
     *   stm  (bit 2) - a transmitted frame counts as valid without a dominant
     *                  acknowledge bit, so no second node is needed.
     *   ilbp (bit 21) - "When enabled, CTU CAN FD receives any frame it
     *                  transmits." This is the one that closes the loop inside
     *                  the chip, so nothing has to be wired between the pads.
     *
     * Measured with stm but not ilbp: TX buffers drained, RX stayed empty and
     * the buffer ended in state 6 = TXT_ERR at all four bit rates.
     */
    twaifd_ll_set_mode(s_twai, false, true, true);
    twaifd_ll_enable_test_mode(s_twai, true);

    /*
     * afm = 0: store every received frame.
     *
     * The register description is explicit - "If enabled, only RX frames which
     * pass Frame filters are stored in RX buffer; if disabled, every received
     * frame is stored". With afm = 1 and no filters programmed, every frame is
     * dropped, which looks exactly like "TX does not loop back". The self test
     * wants to see the frame, not to exercise the filters.
     */
    twaifd_ll_enable_filter_mode(s_twai, false);
    twaifd_ll_enable_fd_mode(s_twai, true);
    twaifd_ll_enable_rxfifo_auto_increase(s_twai, true);
    twaifd_ll_enable_bus_off_tx_fail_mode(s_twai, true);
    twaifd_ll_filter_drop_rtr(s_twai, false);
    twaifd_ll_set_tx_retrans_limit(s_twai, 3);

    if (s31_twai_timing(500000u, &timing) == RT_EOK)
    {
        twaifd_ll_set_nominal_bitrate(s_twai, &timing);
    }

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_twai_init);

rt_err_t rt_hw_twai_set_bitrate(rt_uint32_t bitrate)
{
    twai_timing_advanced_config_t timing;
    rt_err_t err;

    if (s_twai == RT_NULL)
    {
        return -RT_ERROR;
    }

    err = s31_twai_timing(bitrate, &timing);
    if (err != RT_EOK)
    {
        return err;
    }

    /* Timing can only be changed while the controller is off the bus. */
    twaifd_ll_enable_hw(s_twai, false);
    twaifd_ll_set_nominal_bitrate(s_twai, &timing);
    twaifd_ll_enable_hw(s_twai, true);

    /*
     * Clear the error counters, with a bound.
     *
     * twaifd_ll_set_operate_cmd() is
     *     hw->command.val = commands;
     *     while (hw->command.val & commands);
     * - a bare spin with no timeout. If the bit does not self-clear the board
     * stops responding entirely, which is exactly what happened after a
     * successful first frame: the second bit rate never printed and the shell
     * died. Issuing the command directly and polling with a cycle budget keeps
     * a misbehaving hardware handshake from taking the console with it.
     */
    s_twai->command.val = TWAIFD_LL_HW_CMD_RST_ERR_CNT;
    {
        rt_uint32_t start = s31_twai_cycles();

        while (s_twai->command.val & TWAIFD_LL_HW_CMD_RST_ERR_CNT)
        {
            if ((rt_uint32_t)(s31_twai_cycles() - start) >
                (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000u * 5u))
            {
                break;      /* the counters are diagnostics, not correctness */
            }
        }
    }

    return RT_EOK;
}

rt_err_t rt_hw_twai_set_pins(rt_base_t tx_pin, rt_base_t rx_pin)
{
    if (((SOC_GPIO_VALID_GPIO_MASK >> tx_pin) & 1ULL) == 0 ||
        ((SOC_GPIO_VALID_GPIO_MASK >> rx_pin) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    /*
     * Same pad for TX and RX is a supported configuration here, and it is how
     * the controller's own loopback was checked.
     *
     * With a single pad the pad's input sees what its output drives, so the
     * frame goes out and comes straight back without any jumper. That separates
     * two failures that look identical from the outside: if this works, the
     * controller's internal loopback (ilbp) is what is not taking effect; if it
     * does not, the frame never reaches the pin at all.
     */

    /* TX: peripheral drives the pad. */
    gpio_ll_func_sel(&GPIO, (uint8_t)tx_pin, PIN_FUNC_GPIO);
    gpio_ll_output_enable(&GPIO, (uint8_t)tx_pin);
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)tx_pin, true, false);
    esp_rom_gpio_connect_out_signal((uint32_t)tx_pin, TWAI0_TX_IDX, false, false);

    /*
     * RX: pad feeds the peripheral, with a pull-up.
     *
     * The pull-up is not cosmetic. A CAN bus idles recessive (high) and the
     * controller has to see 11 consecutive recessive bits before it will join
     * the bus; a floating input is read as whatever noise arrives, so the node
     * never becomes error-active and every frame fails.
     *
     * Measured before the pull-up: TX buffer ended in state 6 (TXT_ERR,
     * "Failed") with the error counters at zero and nothing received.
     *
     * On a real board with a transceiver the pull-up is harmless - the
     * transceiver drives the pin - so this is correct in both configurations.
     */
    gpio_ll_func_sel(&GPIO, (uint8_t)rx_pin, PIN_FUNC_GPIO);
    gpio_ll_input_enable(&GPIO, (uint8_t)rx_pin);
    gpio_ll_output_disable(&GPIO, (uint8_t)rx_pin);
    gpio_ll_pullup_en(&GPIO, (uint8_t)rx_pin);
    gpio_ll_pulldown_dis(&GPIO, (uint8_t)rx_pin);
    esp_rom_gpio_connect_in_signal((uint32_t)rx_pin, TWAI0_RX_IDX, false);

    return RT_EOK;
}

/*
 * Queue one classic CAN frame.
 *
 * A classic frame is an FD-capable frame buffer with fdf = 0 and brs = 0, which
 * is what the "fd mode enabled, frame controlled by header.fdf" setting in
 * init means: the block can do FD, but it sends exactly what the header asks
 * for.
 */
rt_err_t rt_hw_twai_send(rt_uint32_t id, const rt_uint8_t *data, rt_uint32_t len,
                         rt_uint32_t timeout_ms)
{
    twai_frame_header_t hdr;
    twaifd_frame_buffer_t buf;
    rt_uint32_t buf_idx = 0;
    rt_uint32_t start;

    if (s_twai == RT_NULL || len > 8u || data == RT_NULL)
    {
        return -RT_EINVAL;
    }

    rt_memset(&hdr, 0, sizeof(hdr));
    hdr.id  = id & 0x7FFu;      /* standard 11-bit identifier */
    hdr.dlc = (rt_uint16_t)len;
    hdr.ide = 0;
    hdr.rtr = 0;
    hdr.fdf = 0;                /* classic CAN */
    hdr.brs = 0;

    rt_memset(&buf, 0, sizeof(buf));
    twaifd_ll_format_frame_header(&hdr, (rt_uint8_t)len, &buf);
    twaifd_ll_format_frame_data((const rt_uint8_t *)data, len, &buf);

    /*
     * `rwcnt` is deliberately left alone: despite the name it is the
     * RETRANSMISSION counter (twaifd_struct.h: "Re-transmission counter"), not a
     * word count. Writing (len+3)/4 into it changes how many times the frame is
     * retried, which is not what a formatter should touch.
     *
     * The DLC written by twaifd_ll_format_frame_header() is the length field the
     * controller uses for a classic frame.
     */

    /*
     * Wait for a free TX buffer, using the state encoding instead of comparing
     * to zero.
     *
     * The status field is a 4-bit state code whose reset value is 8, not 0 - so
     * the obvious `while (status != 0)` never terminates. Measured: the first
     * run reported "TX buffer never became free" at all four bit rates.
     *
     * 8 is the "empty" state. The field is read through the LL accessor, which
     * shifts and masks it out of the shared tx_status register.
     */
    start = s31_twai_cycles();
    while (!s31_twai_buffer_free(buf_idx))
    {
        buf_idx++;

        if (buf_idx >= s_twai_buffer_count)
        {
            buf_idx = 0;

            if ((rt_uint32_t)(s31_twai_cycles() - start) >
                (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000u * timeout_ms))
            {
                return -RT_ETIMEOUT;
            }
        }
    }

    /*
     * Return the buffer to "Empty" first.
     *
     * A buffer that finished a transmission sits in TX_OK (4) or Failed (6), and
     * neither state accepts a new frame - only "Empty" (8) does. Without this the
     * next send mounts bytes into a buffer the controller is not watching, and
     * the state never changes.
     *
     * Measured: a second round trip at the same bit rate reported "TX buffer
     * never became free" while the first one transmitted.
     */
    twaifd_ll_set_tx_buffer_cmd(s_twai, (rt_uint8_t)buf_idx,
                                TWAIFD_LL_TX_CMD_EMPTY);

    twaifd_ll_mount_tx_buffer(s_twai, &buf, (rt_uint8_t)buf_idx);
    twaifd_ll_set_tx_buffer_cmd(s_twai, (rt_uint8_t)buf_idx,
                                TWAIFD_LL_TX_CMD_READY);

    return RT_EOK;
}

/* Receive one frame, if one is waiting. */
rt_err_t rt_hw_twai_recv(rt_uint32_t *id, rt_uint8_t *data, rt_uint32_t *len,
                         rt_uint32_t timeout_ms)
{
    twaifd_frame_buffer_t buf;
    twai_frame_header_t hdr;
    rt_uint32_t start;

    if (s_twai == RT_NULL || id == RT_NULL || data == RT_NULL || len == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*
     * Wait on the FIFO's own frame counter.
     *
     * This used to accept EITHER "the empty flag is clear" OR "RBNEI is set".
     * RBNEI is an interrupt status bit and it does not track the read pointer:
     * once asserted it stays asserted until cleared, and clearing it is not
     * sufficient either, since a frame the FIFO has already handed out can
     * re-assert it. Waiting on it therefore wakes up when there is nothing to
     * read.
     *
     * IDF never makes this decision from RBNEI. esp_twai_onchip.c drains with
     *
     *     while (twai_hal_get_rx_msg_count(twai_ctx->hal)) { ... }
     *
     * and twai_hal_get_rx_msg_count() is exactly twaifd_ll_get_rx_frame_count().
     * That value is the FIFO's count, so it cannot report a frame that is not
     * there.
     */
    start = s31_twai_cycles();
    while (twaifd_ll_get_rx_frame_count(s_twai) == 0u)
    {
        if ((rt_uint32_t)(s31_twai_cycles() - start) >
            (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000u * timeout_ms))
        {
            return -RT_ETIMEOUT;
        }
    }

    twaifd_ll_get_rx_frame(s_twai, &buf);

    /*
     * Clear RBNEI after the copy, not before.
     *
     * IDF's driver says it outright (esp_hal_twai/twai_hal_v2.c):
     *
     *     twaifd_ll_get_rx_frame(dev, rx_frame);
     *     // 'not_empty' interrupt can only be cleared after reading the frame
     *     twaifd_ll_clr_intr_status(dev, TWAIFD_LL_INTR_RX_NOT_EMPTY);
     *
     * Leaving it set makes the interrupt look permanently pending, so an
     * interrupt-driven receive one day written on top of this would spin.
     */
    twaifd_ll_clr_intr_status(s_twai, TWAIFD_LL_INTR_RX_NOT_EMPTY);

    twaifd_ll_parse_frame_header(&buf, &hdr);

    /*
     * A classic frame's DLC is its byte count and cannot exceed 8.
     *
     * The previous code called twaifd_dlc2len() and then clamped the result to 8,
     * which turned an impossible header into a plausible-looking read: a frame
     * whose format word decoded as DLC 9 (measured once, w0=0x00002909) was
     * reported as an 8-byte frame and returned RT_EOK. A driver that can see the
     * header is illegal must say so - returning the nearest legal value hides
     * exactly the corruption the caller needs to know about.
     *
     * The bound is checked here rather than relying on twaifd_dlc2len()'s own
     * HAL_ASSERT, because an assert on an RX path is a hang.
     */
    if (hdr.dlc > 8u)
    {
        rt_kprintf("twai: illegal DLC %u in received frame (w0=0x%08x)\n",
                   (unsigned)hdr.dlc, (unsigned)buf.words[0]);
        return -RT_ERROR;
    }

    *id  = hdr.id;
    *len = twaifd_dlc2len((rt_uint16_t)hdr.dlc);

    twaifd_ll_parse_frame_data(&buf, data, (int)*len);

    return RT_EOK;
}

rt_err_t rt_hw_twai_start(void)
{
    rt_uint32_t start;

    if (s_twai == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* Clear any stale state-change flag before enabling, so the wait below
     * cannot be satisfied by a leftover. */
    twaifd_ll_clr_intr_status(s_twai, TWAIFD_LL_INTR_FSM_CHANGE);

    twaifd_ll_enable_hw(s_twai, true);

    /*
     * Wait for the fault-confinement state to actually change.
     *
     * CTU CAN FD needs 11 consecutive recessive bits before it joins the bus,
     * and until then it is not ready to transmit. Arming a frame in that window
     * fails it immediately: measured, the TX buffer went from Ready (1) to
     * Failed (6) inside the send call, and the error counters stayed at zero
     * because the frame never reached the wire.
     *
     * The hardware signals the transition with the FCSI bit. twaifd_ll exposes
     * twaifd_ll_waiting_state_change() as a bare spin with no timeout, which is
     * unusable from a shell command - if the transition never comes, the board
     * hangs. This is the same wait with a bound.
     */
    start = s31_twai_cycles();
    while (!s_twai->int_stat.fcsi_int_st)
    {
        if ((rt_uint32_t)(s31_twai_cycles() - start) >
            (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000u * 20u))
        {
            /* Not fatal: the controller may already be in a settled state and
             * simply have no fresh transition to report. Report it so a caller
             * can tell the two apart. */
            return -RT_ETIMEOUT;
        }
    }

    twaifd_ll_clr_intr_status(s_twai, TWAIFD_LL_INTR_FSM_CHANGE);

    return RT_EOK;
}

void rt_hw_twai_stop(void)
{
    if (s_twai != RT_NULL)
    {
        twaifd_ll_enable_hw(s_twai, false);
    }
}

rt_uint32_t s31_twai_dbg_tx_status(void)
{
    return s_twai ? s_twai->tx_status.val : 0u;
}

rt_uint32_t s31_twai_dbg_rx_count(void)
{
    return s_twai ? twaifd_ll_get_rx_frame_count(s_twai) : 0u;
}

int s31_twai_dbg_rx_empty(void)
{
    return (s_twai && twaifd_ll_is_rx_buffer_empty(s_twai)) ? 1 : 0;
}

rt_uint32_t s31_twai_dbg_tec(void)
{
    return s_twai ? twaifd_ll_get_tec(s_twai) : 0u;
}

rt_uint32_t s31_twai_dbg_rec(void)
{
    return s_twai ? twaifd_ll_get_rec(s_twai) : 0u;
}

int s31_twai_dbg_state(void)
{
    return s_twai ? (int)s_twai->mode_settings.val : -1;
}

rt_uint32_t s31_twai_dbg_intr(void)
{
    return s_twai ? twaifd_ll_get_intr_status(s_twai) : 0u;
}

rt_uint32_t s31_twai_dbg_mode(void)
{
    return s_twai ? s_twai->mode_settings.val : 0u;
}

#endif /* SOC_TWAI_SUPPORTED */
