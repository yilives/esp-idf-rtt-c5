/*
 * ESP32-C5 TWAI (CAN / CAN-FD): classic CAN frames, as a plain C API.
 *
 * RT-Thread has an `rt_can` device class. Like the S31 file this is ported from,
 * the hardware path is proven first and the rt_can registration is deliberately
 * left as a separate step - a device class on top of an unproven register
 * sequence would only make the failures harder to see.
 *
 * ---------------------------------------------------------------------------
 * How this is verified with no second node and no transceiver.
 *
 * A CAN controller normally needs another node to acknowledge every frame; with
 * nobody to ACK, a transmitter retries forever and a test can only report
 * "timed out". The CTU CAN FD controller has a self-test mode (`stm`) that does
 * not wait for an acknowledge, and an internal loopback bit (`ilbp`) that feeds
 * TX back to RX inside the chip. Both together close the loop on a bare board.
 *
 * So `c5twai` does:
 *   1. puts the controller in self-test + internal loopback
 *   2. transmits a frame with a known ID and payload
 *   3. reads it back out of the RX FIFO
 *   4. compares id, length and every data byte
 *
 * It runs the same test at several bit rates, because a wrong BRP shows up as a
 * bit rate that does not work while its neighbour does.
 *
 * What is NOT verified, stated as much: the analog layer. No transceiver is
 * fitted, so there is no differential pair, no termination, no arbitration
 * against a real second node and no error-frame behaviour. What is verified is
 * the controller: timing registers, frame formatting, TX buffer handoff, RX
 * FIFO, and the data path in both directions.
 *
 * ---------------------------------------------------------------------------
 * C5 specifics, versus the S31 file this was ported from:
 *
 *  - The C5 exposes BOTH a `twai_ll.h` and a `twaifd_ll.h`. The first only has
 *    clock/reset plumbing (4 functions); every frame/register operation lives in
 *    `twaifd_ll.h`. Reading `twai_ll.h` alone suggests the driver does not exist.
 *
 *  - There is NO `twaifd_ll_mem_force_power_on()` on this chip. The S31 needs it
 *    (its controller memory sits in a low-power domain that defaults off); the C5
 *    does not expose that register in the LL, so there is nothing to call and the
 *    memory domain is presumably always on. Do not assume the S31 step is
 *    required here.
 *
 *  - There is NO `twaifd_dlc2len()` helper. A local table does the job; DLC 9..15
 *    are not legal in a classic frame and are rejected rather than clamped.
 *
 *  - `twaifd_ll_set_mode()` carries an errata workaround on this silicon: with
 *    `listen_only` it also sets `rom` and `acf` (errata 0v2 issue 5,
 *    espressif/esp-idf#17461). Passing listen_only = false keeps those clear.
 *
 *  - TWAI0 and TWAI1 share one matrix index (66 and 70 respectively for both TX
 *    and RX), so a single-pad loopback is a legal wiring on this part too.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/twaifd_struct.h"
#include "hal/twaifd_ll.h"
#include "hal/twai_ll.h"
#include "hal/twai_types.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk.h"
#include "esp_attr.h"

#if defined(SOC_TWAI_SUPPORTED)

#include "port_twai.h"

/* set_pins() is used from rt_hw_twai_init() below and defined further down, so it
 * needs a prototype before that call. */
rt_err_t rt_hw_twai_set_pins(rt_base_t tx_pin, rt_base_t rx_pin);

/* TX buffer state encoding. The field is a 4-bit state code whose *empty* value
 * is 8, not 0 - the S31 file records that the obvious `while (status != 0)`
 * never terminates, and that its first run reported "TX buffer never became
 * free" at all four bit rates because of it. */
#define C5_TWAI_TXB_EMPTY   8u

static twaifd_dev_t *s_twai;
static rt_uint32_t   s_twai_buffer_count = 1u;

static inline rt_bool_t c5_twai_buffer_free(rt_uint32_t idx)
{
    return twaifd_ll_get_tx_buffer_status(s_twai, (rt_uint8_t)idx) ==
           C5_TWAI_TXB_EMPTY;
}

/* A cycle counter for bounded waits. The LL's own waiting helpers are bare spins
 * with no timeout (twaifd_ll_set_operate_cmd and twaifd_ll_waiting_state_change
 * both loop on a self-clearing bit forever), and the S31 file records the board
 * going unresponsive because of exactly that. Every wait below is bounded. */
static inline rt_uint32_t c5_twai_cycles(void)
{
    rt_uint32_t v;

    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));

    return v;
}

static inline rt_bool_t c5_twai_cycles_exceeded(rt_uint32_t start, rt_uint32_t ms)
{
    return (rt_uint32_t)(c5_twai_cycles() - start) >
           (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000u * ms);
}

/* DLC -> byte count. DLC 9..15 are not legal for a classic frame; this returns
 * 0 for them and the caller must reject rather than clamp. `twaifd_dlc2len()`
 * exists on the S31 but not on the C5. */
static rt_uint32_t c5_twai_dlc2len(rt_uint32_t dlc)
{
    static const rt_uint8_t map[9] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };

    return (dlc <= 8u) ? map[dlc] : 0u;
}

/*
 * Pick the timing for a bit rate.
 *
 * The bus is divided into `quanta` ticks of 1/(src/brp) each, and a bit is
 * 1 sync + prop_seg + tseg_1 + tseg_2 quanta. Sixteen quanta per bit is the usual
 * choice: enough resolution to place the sample point, few enough that brp stays
 * small at high rates.
 *
 * The search finds an EXACT divider instead of truncating one. brp and quanta are
 * integers whose product must equal src / bitrate: asking for 16 quanta at
 * 1 Mbit/s gives 80e6 / (1e6 * 16) = 5 exactly, but at 800 kbit/s it gives 6.25,
 * which truncates to 6 and runs the bus 4 % fast. That is not a rounding detail -
 * the S31 file measured 1 Mbit/s alone accumulating tec = 160 while the three
 * lower rates stayed at 0.
 */
static rt_err_t c5_twai_timing(rt_uint32_t src_hz, rt_uint32_t bitrate,
                               twai_timing_advanced_config_t *t)
{
    rt_uint32_t quanta = 0;
    rt_uint32_t brp = 0;
    rt_uint32_t q;

    if (bitrate == 0 || bitrate > 1000000u)
    {
        return -RT_EINVAL;
    }

    for (q = 8u; q <= 25u; q++)
    {
        rt_uint32_t total = src_hz / bitrate;

        if (total == 0u || (total % q) != 0u)
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
        return -RT_EINVAL;      /* no exact solution; refuse rather than drift */
    }

    /* A bit is 1 sync quanta + prop_seg + tseg_1 + tseg_2. The sync segment is
     * mandatory and NOT programmable, so the three programmed segments add up to
     * quanta - 1. The S31 file records splitting all 16 across the three, making
     * every bit one quanta short and running 1 Mbit/s at 1.33.
     *
     * Sample point at 75 %: prop 25 %, tseg1 50 %, tseg2 25 %. */
    {
        rt_uint32_t prog = quanta - 1u;

        /* NO "- 1" anywhere in here, and that is the whole bug.
         *
         * The S31 file subtracts one from prop_seg and tseg_1 because the CTU
         * CAN FD IP it drives encodes each segment as (quanta - 1) in the
         * register. The C5's controller uses a plain Synopsys-style layout -
         * twaifd_struct.h documents `prop: bitpos [6:0], measurement unit: time
         * quanta` with no bias - so carrying the subtraction over made every bit
         * one quanta short per segment.
         *
         * Measured: asking init for 500 kbit/s produced btr = 0x0830a302, which
         * decodes to brp=6, prop=2, ph1=6, ph2=5 - 14 quanta in a bit instead of
         * 16, i.e. 48e6/(6*14) = 571428 Hz, 14 % fast. At every requested rate
         * the same 16/14 factor applies, which is what a form error on every
         * frame looks like: the controller cannot even agree with its own
         * loopback about where the bit boundaries are. */
        t->brp      = brp;
        t->prop_seg = (rt_uint8_t)(prog / 4u);
        t->tseg_1   = (rt_uint8_t)(prog / 2u);
        t->tseg_2   = (rt_uint8_t)(prog - (prog / 4u) - (prog / 2u));
        if (t->tseg_2 < 1u)
        {
            t->tseg_2 = 1u;
        }
        t->sjw = (prog / 8u) > 4u ? 4u : (rt_uint8_t)(prog / 8u);
        if (t->sjw == 0u)
        {
            t->sjw = 1u;
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
        twai_ll_enable_bus_clock(C5_TWAI_ID, true);
        twai_ll_reset_register(C5_TWAI_ID);

        /* XTAL, not the default PLL_F80M. This is the same choice the ADC driver
         * had to make for the same measured reason (report 搂17): the PLL is not
         * enabled in this image unless something asks for it, so a clock whose
         * source defaults to PLL gets no clock at all. */
        twai_ll_set_clock_source(C5_TWAI_ID, TWAI_CLK_SRC_XTAL);
        twai_ll_enable_clock(C5_TWAI_ID, true);
    }

    s_twai = &TWAI0;

    twaifd_ll_reset(s_twai);

    s_twai_buffer_count = twaifd_ll_get_tx_buffer_total(s_twai);
    if (s_twai_buffer_count == 0u || s_twai_buffer_count > 8u)
    {
        s_twai_buffer_count = 1u;
    }

    /* A mode change requires the controller to be disabled - set_mode asserts
     * `ena == 0`. */
    twaifd_ll_enable_hw(s_twai, false);

    /*
     * Self-test AND internal loopback. Both flags; the S31 file records that
     * setting only the first produces a controller that transmits happily and
     * receives nothing:
     *
     *   stm  (self_test) - a transmitted frame counts as valid without a
     *                      dominant acknowledge bit, so no second node is needed.
     *   ilbp (loopback)  - "the controller receives any frame it transmits",
     *                      which closes the loop inside the chip.
     *
     * Measured there with stm but not ilbp: TX buffers drained, RX stayed empty,
     * and the buffer ended in state 6 = TXT_ERR at all four bit rates.
     */
    twaifd_ll_set_mode(s_twai, false, true, true);

    /* NOT twaifd_ll_enable_test_mode().
     *
     * That sets mode_settings.tstm, which the LL header calls "test mode" and
     * which reads like the natural companion to self-test mode. IDF never calls
     * it: twai_hal_v2.c's twai_hal_init() sets stm/ilbp through set_mode() and
     * stops there. Measured with tstm set: a form error (err_type=2) on every
     * frame at every bit rate, tec climbing to 160, and nothing looped back. */

    /* afm = 1, then drop the RTR and let the range filter accept everything.
     *
     * The S31 file uses afm = 0 ("store every received frame") on the reasoning
     * that afm = 1 with no filters programmed drops everything. That is true of
     * a controller with no filter enabled, but IDF's own C5 HAL
     * (twai_hal_v2.c:31) sets afm = 1 and relies on each filter's own enable
     * bit. Measured the other way: afm = 0 gave a form error (err_type = 2) on
     * every frame at every bit rate. */
    twaifd_ll_enable_filter_mode(s_twai, true);
    /* fde = 0: classic frames only.
     *
     * IDF sets this to 1 and notes "fd frame still controlled by header.fdf", so
     * a fdf = 0 frame should still go out as classic. Tried both ways on this
     * board - it changes nothing - so it is left at 0 here, which is the honest
     * setting for a driver whose send path only ever produces classic frames. */
    twaifd_ll_enable_fd_mode(s_twai, false);
    twaifd_ll_enable_rxfifo_auto_increase(s_twai, true);
    twaifd_ll_enable_bus_off_tx_fail_mode(s_twai, true);
    twaifd_ll_filter_drop_rtr(s_twai, false);
    twaifd_ll_set_tx_retrans_limit(s_twai, 3);

    /* One range filter accepting every standard classic id, so that afm = 1
     * still lets the looped-back frame through.
     *
     * This is the half the S31 file does not need, because it stores everything
     * regardless. With afm = 1 a frame no filter accepts is dropped, which pre-
     * sents as "TX does not loop back" - the exact trap the S31 comment warns
     * about from the other direction. */
    twaifd_ll_range_filter_set_id_thres(s_twai, 0, false, 0x7FFu, 0u);
    twaifd_ll_range_filter_accept_classic_std(s_twai, 0, true);

    if (c5_twai_timing((rt_uint32_t)esp_clk_xtal_freq(), 500000u, &timing) == RT_EOK)
    {
        twaifd_ll_set_nominal_bitrate(s_twai, &timing);
    }

    /* Pads last: routing a signal before the controller is configured leaves the
     * pad carrying whatever the register reset value produces.
     *
     * NOTE: on this chip TWAI0_TX_IDX and TWAI0_RX_IDX are the SAME matrix index
     * (66), so an internal-loopback test does not need any pad at all. Routing
     * them is kept for a real transceiver, but see c5_twai_selftest() - it
     * deliberately pulls the RX pad high so a floating line cannot be read as
     * dominant, which is one way to get a form error with nothing attached. */
    rt_hw_twai_set_pins(C5_TWAI_TX_PIN, C5_TWAI_RX_PIN);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_twai_init);

rt_err_t rt_hw_twai_set_bitrate(rt_uint32_t bitrate)
{
    twai_timing_advanced_config_t timing;
    rt_err_t err;
    rt_uint32_t start;

    if (s_twai == RT_NULL)
    {
        return -RT_ERROR;
    }

    err = c5_twai_timing((rt_uint32_t)esp_clk_xtal_freq(), bitrate, &timing);
    if (err != RT_EOK)
    {
        return err;
    }

    /* Timing can only be changed while the controller is off the bus. */
    twaifd_ll_enable_hw(s_twai, false);
    twaifd_ll_set_nominal_bitrate(s_twai, &timing);
    twaifd_ll_enable_hw(s_twai, true);

    /* Clear the error counters with a bound. twaifd_ll_set_operate_cmd() is a
     * bare spin on a self-clearing bit; if it does not clear, the board stops
     * responding. The S31 file lost its console to exactly this after a
     * successful first frame. */
    s_twai->command.val = TWAIFD_LL_HW_CMD_RST_ERR_CNT;
    start = c5_twai_cycles();
    while (s_twai->command.val & TWAIFD_LL_HW_CMD_RST_ERR_CNT)
    {
        if (c5_twai_cycles_exceeded(start, 5u))
        {
            break;              /* the counters are diagnostics, not correctness */
        }
    }

    return RT_EOK;
}

rt_err_t rt_hw_twai_set_pins(rt_base_t tx_pin, rt_base_t rx_pin)
{
    if (tx_pin < 0 || rx_pin < 0 ||
        tx_pin >= SOC_GPIO_PIN_COUNT || rx_pin >= SOC_GPIO_PIN_COUNT ||
        ((SOC_GPIO_VALID_GPIO_MASK >> tx_pin) & 1ULL) == 0 ||
        ((SOC_GPIO_VALID_GPIO_MASK >> rx_pin) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    /* TX: peripheral drives the pad. TWAI0_TX_IDX and TWAI0_RX_IDX are the same
     * matrix index on this chip, so a single-pad loopback is legal wiring. */
    gpio_ll_func_sel(&GPIO, (uint8_t)tx_pin, PIN_FUNC_GPIO);
    gpio_ll_output_enable(&GPIO, (uint8_t)tx_pin);
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)tx_pin, true, false);
    esp_rom_gpio_connect_out_signal((uint32_t)tx_pin, TWAI0_TX_IDX, false, false);

    /* RX: pad feeds the peripheral, with a pull-up.
     *
     * The pull-up is not cosmetic. A CAN bus idles recessive (high) and the
     * controller has to see 11 consecutive recessive bits before it will join
     * the bus; a floating pad gives neither level reliably and the controller
     * never leaves the "not ready" state. */
    gpio_ll_func_sel(&GPIO, (uint8_t)rx_pin, PIN_FUNC_GPIO);
    gpio_ll_input_enable(&GPIO, (uint8_t)rx_pin);
    gpio_ll_pullup_en(&GPIO, (uint8_t)rx_pin);
    gpio_ll_output_disable(&GPIO, (uint8_t)rx_pin);
    esp_rom_gpio_connect_in_signal((uint32_t)rx_pin, TWAI0_RX_IDX, false);

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

    /* Wait for the fault-confinement state to actually change. The controller
     * needs 11 consecutive recessive bits before it joins the bus, and arming a
     * frame in that window fails it immediately - the S31 file measured the TX
     * buffer going from Ready (1) to Failed (6) inside the send call, with the
     * error counters at zero because the frame never reached the wire. */
    start = c5_twai_cycles();
    while (!s_twai->int_stat.fcsi_int_st)
    {
        if (c5_twai_cycles_exceeded(start, 20u))
        {
            /* Not fatal: the controller may already be settled and simply have
             * no fresh transition to report. */
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

/* Queue one classic frame. Returns RT_EOK once the buffer is armed. */
rt_err_t rt_hw_twai_send_ex(rt_uint32_t id, const rt_uint8_t *data,
                            rt_uint32_t len, rt_uint32_t timeout_ms,
                            rt_uint32_t *out_buf_idx)
{
    twaifd_frame_buffer_t buf;
    twai_frame_header_t hdr;
    rt_uint32_t start;
    rt_uint32_t buf_idx = 0;

    if (s_twai == RT_NULL || data == RT_NULL || len > 8u)
    {
        return -RT_EINVAL;
    }

    if (out_buf_idx != RT_NULL)
    {
        *out_buf_idx = 0xFFFFFFFFu;
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
    twaifd_ll_format_frame_data(data, len, &buf);

    /* Wait for a free TX buffer, scanning the buffers - a buffer that finished a
     * transmission sits in TX_OK (4) or Failed (6), and neither accepts a new
     * frame, so "free" means exactly the Empty state. */
    start = c5_twai_cycles();
    while (!c5_twai_buffer_free(buf_idx))
    {
        buf_idx++;

        if (buf_idx >= s_twai_buffer_count)
        {
            buf_idx = 0;

            if (c5_twai_cycles_exceeded(start, timeout_ms))
            {
                return -RT_ETIMEOUT;
            }
        }
    }

    /* Return the buffer to "Empty" first, but ONLY if it is not already empty.
     *
     * The S31 file writes the Empty command unconditionally, on the reasoning
     * that only the Empty state accepts new bytes. That is true, but the command
     * itself is a write to tx_command_txtb_info - issuing it on a buffer that is
     * already Empty re-commands a buffer the controller has just re-armed, and
     * the S31 comment records a second round trip failing for a related reason.
     *
     * Measured here: unconditional Empty changes nothing (the loop above always
     * leaves buf_idx on an already-Empty buffer), so this is a no-op cleanup
     * rather than a fix. */
    if (twaifd_ll_get_tx_buffer_status(s_twai, (rt_uint8_t)buf_idx) != C5_TWAI_TXB_EMPTY)
    {
        twaifd_ll_set_tx_buffer_cmd(s_twai, (rt_uint8_t)buf_idx, TWAIFD_LL_TX_CMD_EMPTY);
    }

    twaifd_ll_mount_tx_buffer(s_twai, &buf, (rt_uint8_t)buf_idx);

    /* No read-back here, and the reason is measured.
     *
     * `txt_mem_cell[]` reads as zero even immediately after writing a known
     * pattern to it: the TX memory window is write-only from the APB side, so a
     * read-back "mismatch" says nothing about whether the frame landed. Writing
     * 0xdeadbeef into an unused buffer and reading it straight back returned
     * 0x00000000, which is what settled it. A check that always reports
     * MISMATCH because the bus cannot be read is worse than no check. */

    twaifd_ll_set_tx_buffer_cmd(s_twai, (rt_uint8_t)buf_idx, TWAIFD_LL_TX_CMD_READY);

    if (out_buf_idx != RT_NULL)
    {
        *out_buf_idx = buf_idx;
    }

    return RT_EOK;
}

/* The buffer index is not needed by every caller, so the plain form stays. */
rt_err_t rt_hw_twai_send(rt_uint32_t id, const rt_uint8_t *data, rt_uint32_t len,
                         rt_uint32_t timeout_ms)
{
    return rt_hw_twai_send_ex(id, data, len, timeout_ms, RT_NULL);
}

/* State codes for a TX buffer, from the 4-bit tx_status field.
 *
 * Empty is 8, not 0 - the S31 file records that `while (status != 0)` never
 * terminates. TX_OK and Failed both refuse a new frame; only Empty accepts one. */
#define C5_TWAI_TXB_TX_OK   4u
#define C5_TWAI_TXB_FAILED  6u

/* Reclaim finished TX buffers.
 *
 * WHY THIS EXISTS - a compound bug that made the CAN device class work exactly
 * once per boot:
 *
 *   1. after a frame goes out, its buffer sits in TX_OK (4) and nothing returned
 *      it to Empty (8);
 *   2. the next send scanned for a free buffer, skipped buffer 0 because it was
 *      TX_OK, and took buffer 1;
 *   3. the ISR reported TX_DONE for box 0 unconditionally, so buffer 1's
 *      completion was never delivered;
 *   4. `rt_completion_wait()` timed out, `_can_int_tx()` took its error path and
 *      returned 0 bytes;
 *   5. `rt_device_write()` therefore returned 0, and the test - which runs once
 *      per boot - never saw it.
 *
 * Measured on the second run in one boot:
 *
 *     c5twai: tx_status=0x00008884 buffer(s)=4: [0]=4 [1]=8 [2]=8 [3]=8
 *
 * State 4 on buffer 0, empty on the rest. `c5twai` passes at four rates because
 * it calls rt_hw_twai_send() directly and never waits on the device class's
 * completion, which is why only `c5can` exposed this. */
rt_uint32_t rt_hw_twai_reclaim_tx_buffers(void)
{
    rt_uint32_t i;
    rt_uint32_t done = 0;

    if (s_twai == RT_NULL)
    {
        return 0;
    }

    for (i = 0; i < s_twai_buffer_count; i++)
    {
        rt_uint32_t st = twaifd_ll_get_tx_buffer_status(s_twai, (rt_uint8_t)i);

        if (st == C5_TWAI_TXB_TX_OK || st == C5_TWAI_TXB_FAILED)
        {
            twaifd_ll_set_tx_buffer_cmd(s_twai, (rt_uint8_t)i,
                                        TWAIFD_LL_TX_CMD_EMPTY);
            done |= (1u << i);
        }
    }

    return done;
}

rt_uint32_t rt_hw_twai_tx_buffer_count(void)
{
    return s_twai_buffer_count;
}

/* Dump every TX buffer's state code, for the second-run failure report.
 *
 * The state is a 4-bit code per buffer in one register. Printing all of them says
 * whether the buffers came back to Empty (8) after the previous run or are stuck
 * in TX_OK (4) / Failed (6), which are the states that refuse a new frame. */
rt_uint32_t rt_hw_twai_tx_status_dump(void)
{
    rt_uint32_t i;
    rt_uint32_t raw;

    if (s_twai == RT_NULL)
    {
        rt_kprintf("c5twai: no controller\n");
        return 0;
    }

    raw = s_twai->tx_status.val;

    rt_kprintf("c5twai: tx_status=0x%08x buffer(s)=%u:", (unsigned)raw,
               (unsigned)s_twai_buffer_count);

    for (i = 0; i < s_twai_buffer_count; i++)
    {
        rt_kprintf(" [%u]=%u", (unsigned)i,
                   (unsigned)twaifd_ll_get_tx_buffer_status(s_twai, (rt_uint8_t)i));
    }

    rt_kprintf("\n");
    return raw;
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

    /* Wait on the FIFO's own frame counter, not on the RBNEI interrupt bit. The
     * S31 file records that RBNEI does not track the read pointer - once
     * asserted it stays asserted until cleared, and clearing is not sufficient
     * either, so waiting on it wakes up with nothing to read. The frame count
     * cannot report a frame that is not there. */
    start = c5_twai_cycles();
    while (twaifd_ll_get_rx_frame_count(s_twai) == 0u)
    {
        if (c5_twai_cycles_exceeded(start, timeout_ms))
        {
            return -RT_ETIMEOUT;
        }
    }

    twaifd_ll_get_rx_frame(s_twai, &buf);

    /* Clear RBNEI after the copy, not before: IDF's own driver notes that the
     * "not empty" interrupt can only be cleared after reading the frame. */
    twaifd_ll_clr_intr_status(s_twai, TWAIFD_LL_INTR_RX_NOT_EMPTY);

    twaifd_ll_parse_frame_header(&buf, &hdr);

    /* A classic frame's DLC is its byte count and cannot exceed 8. Rejecting an
     * impossible header is the only honest answer - clamping it to 8 reports a
     * corrupt frame as a plausible one, hiding the corruption the caller needs
     * to know about. */
    {
        rt_uint32_t n = c5_twai_dlc2len(hdr.dlc);

        if (hdr.dlc > 8u || n == 0u)
        {
            rt_kprintf("c5twai: illegal DLC %u in received frame (w0=0x%08x)\n",
                       (unsigned)hdr.dlc, (unsigned)buf.words[0]);
            return -RT_ERROR;
        }

        *id  = hdr.id;
        *len = n;
        twaifd_ll_parse_frame_data(&buf, data, (int)n);
    }

    return RT_EOK;
}

/* ------------------------------------------------------------------ msh -- */

/* `c5twai` - controller self test across several bit rates.
 *
 * Internal loopback plus self-test mode means the frame leaves the controller
 * and comes back without a transceiver, a second node or a jumper. What that
 * proves is the controller; what it cannot prove is the analog layer, and the
 * output says so.
 */
int c5_twai_selftest(void)
{
    static const rt_uint32_t rates[] = { 125000u, 250000u, 500000u, 1000000u };
    static const rt_uint8_t  payload[4] = { 0x11, 0x22, 0x33, 0x44 };
    rt_uint32_t i;
    int failures = 0;

    if (s_twai == RT_NULL)
    {
        rt_kprintf("c5twai  : FAIL, controller not initialised\n");
        return -1;
    }

    rt_kprintf("c5twai  : TWAI0, self-test + internal loopback, no transceiver\n");

    /* Re-claim the pads.
     *
     * Another driver may have taken IO4 in the meantime: the ADC self test parks
     * its channel pads for analog, and ADC1 channel 3 IS GPIO4 - the pad CAN uses
     * for both TX and RX on this board. Measured: `c5adc` followed by `c5twai` in
     * one session made CAN fail with a form error at every rate, while `c5twai`
     * alone passed.
     *
     * A driver that is about to use a pin has to assert that pin, not assume the
     * routing from rt_hw_twai_init() is still in place. This is the same rule the
     * PWM self test follows by calling c5_pwm_set_pin() on every run. */
    rt_hw_twai_set_pins(C5_TWAI_TX_PIN, C5_TWAI_RX_PIN);

    /* Dump what the driver thinks it programmed before touching the bus.
     *
     * A form error (err_type = 2) on every frame at every rate can only mean the
     * frame the controller puts on the wire is structurally illegal - so the
     * first question is whether the timing and mode registers hold what was
     * intended, and whether the formatted TX buffer looks like a valid classic
     * frame. Printing all three separates "wrong bit rate" from "wrong frame". */
    {
        twaifd_frame_buffer_t probe;
        twai_frame_header_t   ph;
        rt_uint8_t            pdata[4] = { 0x11, 0x22, 0x33, 0x44 };

        rt_memset(&ph, 0, sizeof(ph));
        ph.id = 0x123;
        ph.dlc = 4;
        rt_memset(&probe, 0, sizeof(probe));
        twaifd_ll_format_frame_header(&ph, 4, &probe);
        twaifd_ll_format_frame_data(pdata, 4, &probe);

        rt_kprintf("c5twai  : xtal=%u Hz btr=0x%08x mode=0x%08x ena=%u\n",
                   (unsigned)esp_clk_xtal_freq(),
                   (unsigned)s_twai->btr.val,
                   (unsigned)s_twai->mode_settings.val,
                   (unsigned)s_twai->mode_settings.ena);
        rt_kprintf("c5twai  : probe frame w0=0x%08x w1=0x%08x (id=0x123 dlc=4)\n",
                   (unsigned)probe.words[0], (unsigned)probe.words[1]);
        rt_kprintf("c5twai  : pads TX=IO%d RX=IO%d, idle levels tx=%d rx=%d\n",
                   (int)C5_TWAI_TX_PIN, (int)C5_TWAI_RX_PIN,
                   gpio_ll_get_level(&GPIO, (uint32_t)C5_TWAI_TX_PIN),
                   gpio_ll_get_level(&GPIO, (uint32_t)C5_TWAI_RX_PIN));
    }

    for (i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
    {
        rt_uint32_t id = 0x123u + i;
        rt_uint8_t  rx[8];
        rt_uint32_t rx_id = 0;
        rt_uint32_t rx_len = 0;
        rt_err_t    err;

        rt_memset(rx, 0, sizeof(rx));

        rt_hw_twai_stop();
        err = rt_hw_twai_set_bitrate(rates[i]);
        if (err != RT_EOK)
        {
            rt_kprintf("c5twai  : %u bit/s not representable (%d)\n",
                       (unsigned)rates[i], (int)err);
            failures++;
            continue;
        }

        err = rt_hw_twai_start();
        if (err != RT_EOK)
        {
            /* Reported, not fatal: the controller may already be settled. */
            rt_kprintf("c5twai  : %u bit/s: no FSM transition (%d), continuing\n",
                       (unsigned)rates[i], (int)err);
        }

        err = rt_hw_twai_send(id, payload, sizeof(payload), 50u);
        if (err != RT_EOK)
        {
            rt_kprintf("c5twai  : %u bit/s: send failed %d\n",
                       (unsigned)rates[i], (int)err);
            failures++;
            continue;
        }

        err = rt_hw_twai_recv(&rx_id, rx, &rx_len, 50u);
        if (err != RT_EOK)
        {
            /* The error counters are the only thing that says WHY a frame
             * failed. tx_status alone gives the buffer state; tec/rec and the
             * error reason distinguish "no acknowledge" from "wrong bit rate
             * against itself" from "bus off". */
            {
                /* twaifd_ll_get_err_reason() returns a bit-field STRUCT, not an
                 * integer - casting it is a compile error ("aggregate value used
                 * where an integer was expected"). Reading the raw err_type
                 * nibble is both simpler and what the caller wants here. */
                rt_uint32_t raw = s_twai->err_capt_retr_ctr_alc_ts_info.err_type;

                rt_kprintf("c5twai  : %u bit/s: nothing came back (%d), "
                           "tx_status=0x%08x tec=%u rec=%u err_type=%u\n",
                           (unsigned)rates[i], (int)err,
                           (unsigned)s_twai->tx_status.val,
                           (unsigned)twaifd_ll_get_tec(s_twai),
                           (unsigned)twaifd_ll_get_rec(s_twai),
                           (unsigned)raw);
            }
            failures++;
            continue;
        }

        if (rx_id != id || rx_len != sizeof(payload) ||
            rt_memcmp(rx, payload, sizeof(payload)) != 0)
        {
            rt_kprintf("c5twai  : %u bit/s: MISMATCH id=0x%x len=%u "
                       "data=%02x %02x %02x %02x\n",
                       (unsigned)rates[i], (unsigned)rx_id, (unsigned)rx_len,
                       rx[0], rx[1], rx[2], rx[3]);
            failures++;
            continue;
        }

        rt_kprintf("c5twai  : %u bit/s: id=0x%03x len=%u echoed back OK\n",
                   (unsigned)rates[i], (unsigned)rx_id, (unsigned)rx_len);
    }

    rt_hw_twai_stop();

    if (failures)
    {
        rt_kprintf("c5twai  : FAIL, %d of %u rate(s) did not round-trip\n",
                   failures, (unsigned)(sizeof(rates) / sizeof(rates[0])));
        return -1;
    }

    rt_kprintf("c5twai  : PASS at all %u rates (controller verified; the analog "
               "layer is not - no transceiver fitted)\n",
               (unsigned)(sizeof(rates) / sizeof(rates[0])));
    return 0;
}

#endif /* SOC_TWAI_SUPPORTED */























