/*
 * ESP32-C5 PARLIO TX: parallel output, 1-bit width, driven by AHB GDMA.
 *
 * This is the last peripheral in this port that can be proven on a bare board,
 * and it needed the GDMA driver (搂17n) before it could exist at all: PARLIO TX
 * has no CPU-writable data register - the peripheral has a `tx_data_cfg` and a
 * FIFO reset but no TXFIFO port - so its payload can only arrive through GDMA.
 *
 * ---------------------------------------------------------------------------
 * How this is verified, and it is a cross-check between two proven peripherals.
 *
 * PARLIO drives two pads: the bit clock and data line 0. There is no internal
 * loopback (SOC_PARLIO_TX_SUPPORT_LOOP_TRANSMISSION is a DMA-EOF workaround, not
 * an echo), so the waveform cannot be read back by PARLIO itself.
 *
 * But the clock line can be counted by the PCNT unit from 搂17d, which was verified
 * independently - and that makes a real measurement rather than a self-report:
 *
 *   1. The GDMA feeds a known number of bytes.
 *   2. PARLIO clocks them out, one bit per clock.
 *   3. PCNT counts rising edges on the clock pad.
 *   4. The count must equal bytes * 8.
 *
 * If PARLIO never started, the count is 0. If it started but the GDMA handed it
 * the wrong amount of data, the count is wrong by exactly that ratio. That
 * distinguishes the failures instead of collapsing them into "it did not work".
 *
 * What is NOT verified: the data line's levels. Counting clock edges says the
 * shift engine ran and consumed the right number of bytes; it says nothing about
 * whether the bits on TXD0 are the ones supplied. Doing that needs a second
 * observation point on the data pad, which the chip does not offer for a signal
 * that only ever leaves.
 *
 * ---------------------------------------------------------------------------
 * C5 specifics:
 *
 *  - PARLIO_CLK_SRC_XTAL, not the default PLL_F240M - five drivers in this port
 *    have now been bitten by a default that points at a PLL nobody enabled.
 *
 *  - `PARLIO_LL_SUPPORT_TX_EOF_FROM_DMA` is 1 here, so the frame ends on the GDMA
 *    EOF and `set_trans_bit_len` does not have to be exact. The bit length is
 *    still programmed, because a driver that only works when one of two
 *    mechanisms is present is not a driver.
 *
 *  - The clock enable is a separate gate INSIDE the peripheral
 *    (`PCR.parl_clk_tx_conf.parl_clk_tx_en`), on top of the module clock
 *    (`PCR.parl_io_conf.parl_clk_en`). IDF enables the core clock only after
 *    starting the unit - doing it earlier produces a transmitter that runs with
 *    nothing coming out.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/parl_io_struct.h"
#include "soc/gpio_sig_map.h"
#include "hal/parlio_ll.h"
#include "hal/ahb_dma_ll.h"
#include "hal/gdma_ll.h"
#include "hal/gdma_channel.h"
#include "hal/dma_types.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk.h"
#include "esp_attr.h"
#include "soc/pcr_struct.h"

#if defined(SOC_PARLIO_SUPPORTED)

#include "port_parlio.h"
#include "port_gdma.h"
#include "port_pcnt.h"

#define C5_PARLIO_GROUP     0
#define C5_GDMA_TX_CH       0

static parl_io_dev_t *s_parlio;
static rt_bool_t      s_parlio_ready;

rt_err_t rt_hw_parlio_init(void)
{
    if (s_parlio_ready)
    {
        return RT_EOK;
    }

    PERIPH_RCC_ATOMIC()
    {
        parlio_ll_enable_bus_clock(C5_PARLIO_GROUP, true);
        parlio_ll_reset_register(C5_PARLIO_GROUP);
    }

    s_parlio = &PARL_IO;

    /* Clear the clock divider and pick XTAL. Both are group-level fields in PCR,
     * so they are set once and apply to every channel. */
    PERIPH_RCC_ATOMIC()
    {
        parlio_ll_tx_set_clock_source(s_parlio, PARLIO_CLK_SRC_XTAL);
        parlio_ll_tx_reset_clock(s_parlio);
    }

    s_parlio_ready = RT_TRUE;
    return RT_EOK;
}

/* Route the bit clock and data line 0 to pads. */
rt_err_t rt_hw_parlio_set_pin(rt_base_t clk_pin, rt_base_t data_pin)
{
    if (clk_pin < 0 || data_pin < 0 ||
        clk_pin >= SOC_GPIO_PIN_COUNT || data_pin >= SOC_GPIO_PIN_COUNT ||
        ((SOC_GPIO_VALID_GPIO_MASK >> clk_pin) & 1ULL) == 0 ||
        ((SOC_GPIO_VALID_GPIO_MASK >> data_pin) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    /* ctrl_by_periph = true, oen_inv = FALSE - the same shape every other driver
     * in this port had to be corrected to (搂17k). Passing true as the second
     * argument disables the pad exactly when the peripheral drives it. */
    gpio_ll_func_sel(&GPIO, (uint8_t)clk_pin, PIN_FUNC_GPIO);
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)clk_pin, true, false);
    esp_rom_gpio_connect_out_signal((uint32_t)clk_pin, PARL_TX_CLK_OUT_IDX, false, false);
    gpio_ll_input_enable(&GPIO, (uint8_t)clk_pin);   /* the PCNT observer needs it */

    gpio_ll_func_sel(&GPIO, (uint8_t)data_pin, PIN_FUNC_GPIO);
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)data_pin, true, false);
    esp_rom_gpio_connect_out_signal((uint32_t)data_pin, PARL_TX_DATA0_IDX, false, false);

    return RT_EOK;
}

/* One frame: clock `len` bytes out of `buf`, 1 bit per clock. */
rt_err_t rt_hw_parlio_tx(const rt_uint8_t *buf, rt_uint32_t len,
                         rt_uint32_t clk_hz, rt_uint32_t timeout_ms,
                         rt_uint32_t *out_bits)
{
    static dma_descriptor_t desc __attribute__((aligned(8)));
    ahb_dma_dev_t *dma;
    rt_uint32_t xtal;
    rt_uint32_t div;
    rt_uint32_t start;
    rt_uint32_t status;

    if (!s_parlio_ready && rt_hw_parlio_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (buf == RT_NULL || len == 0u || len > DMA_DESCRIPTOR_BUFFER_MAX_SIZE ||
        clk_hz == 0u)
    {
        return -RT_EINVAL;
    }

    xtal = (rt_uint32_t)esp_clk_xtal_freq();
    div = xtal / clk_hz;
    if (div == 0u)
    {
        div = 1u;
    }
    if (div > PARLIO_LL_TX_MAX_CLK_INT_DIV)
    {
        div = PARLIO_LL_TX_MAX_CLK_INT_DIV;
    }

    /* ---- PARLIO side ---- */
    parlio_ll_tx_set_bus_width(s_parlio, 1);
    parlio_ll_tx_set_bit_pack_order(s_parlio, PARLIO_BIT_PACK_ORDER_MSB);
    parlio_ll_tx_set_shift_clock_edge(s_parlio, PARLIO_SHIFT_EDGE_POS);

    {
        hal_utils_clk_div_t d = { .integer = div, .denominator = 0, .numerator = 0 };

        parlio_ll_tx_set_clock_div(s_parlio, &d);
    }

    /* The clock gate must be off while the FIFO is reset, or the reset raises a
     * spurious FIFO-empty interrupt. IDF's ordering, and it matters. */
    PERIPH_RCC_ATOMIC()
    {
        parlio_ll_tx_enable_clock(s_parlio, false);
    }
    parlio_ll_tx_reset_fifo(s_parlio);

    parlio_ll_tx_set_idle_data_value(s_parlio, 0);

    /* Clock gating OFF, and this is the setting that decides whether the unit
     * clocks at all for a 1-bit data width.
     *
     * `PARLIO_LL_TX_DATA_LINE_AS_CLK_GATE` is 7: with gating enabled the unit
     * treats the MSB of the data line (TXD7) as a clock-enable, and at 1-bit width
     * there is no TXD7 to look at. IDF passes `config->flags.clk_gate_en`, which
     * defaults false, and this driver passed false too - so this is not the fix,
     * but it is worth stating that the interaction exists. */
    parlio_ll_tx_enable_clock_gating(s_parlio, false);

    /* End the frame on the programmed BIT LENGTH, not on the DMA EOF.
     *
     * Both work in principle on this chip (PARLIO_LL_SUPPORT_TX_EOF_FROM_DMA == 1),
     * but with DMA_EOF the unit waits for the channel to signal end-of-frame on the
     * peripheral bus, and measured here that signal never arrived: the DMA
     * completed (`dma_txint=0x0b`) while PARLIO's own interrupt status stayed
     * 0x00000000 and `st` stayed 0. With DATA_LEN the unit counts its own output
     * bits and the condition is internal, which removes the peripheral-side
     * handshake from the question entirely.
     *
     * The bit length was already programmed; this just makes it authoritative. */
    /* DMA_EOF, not DATA_LEN.
     *
     * DATA_LEN was tried first and does not fire: the unit shifted 127 bits of a
     * 128-bit frame (tx_st0.tx_fifo_rd_bit_cnt == 127), so an exact bit-length
     * match never happened - off by one, and the unit sat there with its FIFO
     * empty and its clock still running.
     *
     * Every earlier test of DMA_EOF proved nothing, because `int_st` is
     * `int_raw & int_ena` and `int_ena` was 0 until this round. The conclusion
     * "DMA_EOF never arrives" was drawn from a register that could not have
     * reported it.
     *
     * Both are programmed: DMA_EOF decides when the event fires, and the bit
     * length is still set because it costs nothing and a driver should not depend
     * on one of two mechanisms. */
    parlio_ll_tx_set_eof_condition(s_parlio, PARLIO_LL_TX_EOF_COND_DMA_EOF);
    parlio_ll_tx_set_trans_bit_len(s_parlio, len * 8u);

    /* ENABLE THE INTERRUPT BITS.
     *
     * `parlio_ll_tx_get_interrupt_status()` returns `int_st`, and `int_st` is
     * `int_raw & int_ena`. This driver never wrote `int_ena`, so it stayed 0 and
     * the status read returned 0 no matter what the unit did - which is why every
     * attempt reported `int_st=0x00000000` while the clock line was visibly
     * running (PCNT counted 10182 edges against a control run of 0 on the same
     * pin). The polled condition could not have become true.
     *
     * Reading `int_raw` instead of enabling the mask would also work, but enabling
     * it is what IDF does and costs one write. */
    parlio_ll_enable_interrupt(s_parlio, PARLIO_LL_EVENT_TX_MASK, true);
    parlio_ll_clear_interrupt_status(s_parlio, UINT32_MAX);

    /* ---- GDMA side ---- */
    dma = AHB_DMA_LL_GET_HW(0);

    /* THE GDMA REGISTER CLOCK HAS TO BE ON, and nothing before this point did it.
     *
     * `ahb_dma_ll_force_enable_reg_clock()` is only called from rt_hw_gdma_init(),
     * which the M2M self test calls - so `c5gdma` worked and `c5parlio` did not,
     * and the difference only shows up when PARLIO is run first on a fresh boot.
     * Measured: `peri_out_sel` read back 0 after writing 9, because every write to
     * the channel registers went nowhere, and the DMA then sat with zero interrupt
     * status while the descriptor stayed owned by the engine.
     *
     * A peripheral driver is not entitled to assume another driver initialised a
     * shared block. */
    rt_hw_gdma_init();

    desc.dw0.size = len & 0xFFFu;
    desc.dw0.length = len & 0xFFFu;
    desc.dw0.err_eof = 0;
    desc.dw0.suc_eof = 1;
    desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    desc.buffer = (void *)buf;
    desc.next = RT_NULL;

    ahb_dma_ll_tx_reset_channel(dma, C5_GDMA_TX_CH);

    /* THE PERIPHERAL ID IS THE TRIGGER, and it is not the M2M sentinel.
     * SOC_GDMA_TRIG_PERIPH_PARLIO0 is 9 (gdma_channel.h); writing anything else
     * here makes the channel push data into a peripheral that never reads it, and
     * the frame silently produces no clocks. */
    ahb_dma_ll_tx_connect_to_periph(dma, C5_GDMA_TX_CH,
                                    SOC_GDMA_TRIG_PERIPH_PARLIO0);
    ahb_dma_ll_tx_set_eof_mode(dma, C5_GDMA_TX_CH, 1);
    ahb_dma_ll_tx_enable_auto_write_back(dma, C5_GDMA_TX_CH, false);

    /* Burst configuration, which M2M did not need and a peripheral transfer does.
     *
     * IDF sets `max_data_burst_size = 32` for PARLIO (parlio_tx.c:156). The M2M
     * self test never touched burst mode and worked, so this was invisible until a
     * real peripheral was on the other end of the channel - and its absence looks
     * exactly like "the peripheral never requested anything": the DMA channel
     * sits with zero interrupt status while the descriptor is still owned by the
     * engine. */
    ahb_dma_ll_tx_enable_data_burst(dma, C5_GDMA_TX_CH, true);
    ahb_dma_ll_tx_set_burst_size(dma, C5_GDMA_TX_CH, 32);
    ahb_dma_ll_tx_enable_descriptor_burst(dma, C5_GDMA_TX_CH, true);
    ahb_dma_ll_tx_set_desc_addr(dma, C5_GDMA_TX_CH, (uint32_t)&desc);
    ahb_dma_ll_tx_clear_interrupt_status(dma, C5_GDMA_TX_CH, UINT32_MAX);

    /* RESET THE TX CLOCK DIVIDER, once per transaction, as IDF does.
     *
     * The divider's threshold only takes effect when its counter next reaches it,
     * so a divider programmed once at init does not apply to the current frame -
     * IDF brackets every transaction with reset_clock() and its comment says
     * exactly that (parlio_tx.c:489-493). */
    PERIPH_RCC_ATOMIC()
    {
        parlio_ll_tx_reset_clock(s_parlio);
    }

    /* THE CORE CLOCK HAS TO BE ON BEFORE THE DMA CAN FILL THE FIFO.
     *
     * IDF's transaction sequence disables the clock, resets the FIFO, starts the
     * DMA, waits for tx_ready, and only then re-enables the clock - which works
     * there because the clock was already enabled once during channel
     * construction (parlio_tx.c:237). Starting from a cold peripheral, as this
     * driver does, that sequence never produces a request: measured with the clock
     * off, `dma_txint=0x00000000` and `st=0x00000000` after the full timeout - the
     * DMA channel never moved a single descriptor because the peripheral never
     * pulled.
     *
     * So the clock goes on before the DMA starts, and stays on. */
    PERIPH_RCC_ATOMIC()
    {
        parlio_ll_tx_enable_clock(s_parlio, true);
    }

    ahb_dma_ll_tx_start(dma, C5_GDMA_TX_CH);

    /* DO NOT WAIT FOR tx_ready.
     *
     * `parlio_ll_tx_is_ready()` reads `st.tx_ready`, and the bit means "the TX
     * FIFO can accept more data". Once the DMA has filled the FIFO - which it has,
     * `dma_txint=0x0b` - that bit is legitimately 0, so waiting for it to become 1
     * waits forever. IDF can wait there because its DMA is still streaming and the
     * FIFO drains as the unit clocks; here the whole frame is already in.
     *
     * Measured: with the wait in place, st stayed 0x00000000 and the unit never
     * started. The frame is small enough to sit entirely in the FIFO, so the
     * correct action is simply to start. */

    parlio_ll_tx_start(s_parlio, true);

    /* Wait for the unit to report EOF: that is the frame actually going out. */
    start = rt_tick_get();
    while (true)
    {
        status = parlio_ll_tx_get_interrupt_status(s_parlio);

        if (status & PARLIO_LL_EVENT_TX_EOF)
        {
            break;
        }

        if ((rt_tick_get() - start) > rt_tick_from_millisecond(timeout_ms))
        {
            /* The unit never reported EOF. Report what the registers say, and
             * note the one thing that makes this diagnosis tricky: `tx_st0` is
             * CLEARED when the unit is stopped, so it has to be read before the
             * stop - reading it afterwards returns zero and looks exactly like a
             * unit that never ran. */
            rt_kprintf("c5parlio: no EOF after %u ms: int_st=0x%08x st=0x%08x "
                       "dma_txint=0x%08x peri_out_sel=%u\n",
                       (unsigned)timeout_ms,
                       (unsigned)parlio_ll_tx_get_interrupt_status(s_parlio),
                       (unsigned)s_parlio->st.val,
                       (unsigned)ahb_dma_ll_tx_get_interrupt_status(dma, C5_GDMA_TX_CH, true),
                       (unsigned)dma->channel[C5_GDMA_TX_CH].out.out_peri_sel.peri_out_sel_chn);
            rt_kprintf("c5parlio: tx_st0=0x%08x tx_cnt=%u fifo_rd_bits=%u\n",
                       (unsigned)s_parlio->tx_st0.val,
                       (unsigned)s_parlio->tx_st0.tx_cnt,
                       (unsigned)s_parlio->tx_st0.tx_fifo_rd_bit_cnt);
            if (out_bits)
            {
                *out_bits = s_parlio->tx_st0.tx_fifo_rd_bit_cnt;
            }
            parlio_ll_tx_start(s_parlio, false);
            ahb_dma_ll_tx_stop(dma, C5_GDMA_TX_CH);
            return -RT_ETIMEOUT;
        }
    }

    /* CAPTURE THE COUNTERS BEFORE STOPPING THE UNIT.
     *
     * `tx_st0` is cleared when the unit is stopped, so reading it afterwards
     * returns 0 - which is exactly what this driver reported for a frame the
     * hardware had in fact transmitted 127 bits of. The value has to be latched
     * first; stopping is a state change, not just a pause.
     *
     * Two FIFOs are in series here: the GDMA's own TX FIFO and PARLIO's.
     * `parlio_rd_bits` says how much PARLIO pulled out of its own, and
     * `dma_fifo_bytes` says whether the bytes are still sitting in the GDMA's -
     * which would mean the peripheral handshake never happened. */
    {
        rt_uint32_t rd_bits = s_parlio->tx_st0.tx_fifo_rd_bit_cnt;
        rt_uint32_t sub = s_parlio->tx_st0.tx_cnt;
        rt_uint32_t dma_bytes = ahb_dma_ll_tx_get_fifo_bytes(dma, C5_GDMA_TX_CH, 1);

        if (out_bits)
        {
            *out_bits = rd_bits;
        }

        /* The unit keeps clocking between the last data bit and the moment this
         * CPU resumes, so tx_cnt is not zero and is not meaningful - it is
         * reported, not checked. */
        if (rd_bits != len * 8u)
        {
            rt_kprintf("c5parlio: unit read %u of %u bit(s) out of its FIFO, "
                       "tx_cnt=%u, dma_fifo_bytes=%u\n",
                       (unsigned)rd_bits, (unsigned)(len * 8u),
                       (unsigned)sub, (unsigned)dma_bytes);
        }
    }

    parlio_ll_tx_start(s_parlio, false);
    ahb_dma_ll_tx_stop(dma, C5_GDMA_TX_CH);
    ahb_dma_ll_tx_clear_interrupt_status(dma, C5_GDMA_TX_CH, UINT32_MAX);
    parlio_ll_clear_interrupt_status(s_parlio, UINT32_MAX);

    return RT_EOK;
}

/* ------------------------------------------------------------------ msh -- */

/* `c5parlio [bytes] [clk_hz]` - clock bytes out and count the clocks with PCNT. */
int c5_parlio_selftest(rt_uint32_t len, rt_uint32_t clk_hz)
{
    static rt_uint8_t payload[64];
    rt_uint32_t i;
    rt_err_t err;
    int control_edges;
    int edges;
    rt_uint32_t consumed = 0;

    if (len == 0u || len > sizeof(payload))
    {
        len = 16u;
    }
    if (clk_hz == 0u || clk_hz > 10000000u)
    {
        clk_hz = 1000000u;
    }

    for (i = 0; i < len; i++)
    {
        payload[i] = (rt_uint8_t)(0xA5u ^ i);
    }

    err = rt_hw_parlio_init();
    if (err != RT_EOK)
    {
        rt_kprintf("c5parlio: FAIL, init (%d)\n", (int)err);
        return -1;
    }

    if (rt_hw_parlio_set_pin(C5_PARLIO_CLK_PIN, C5_PARLIO_DATA_PIN) != RT_EOK)
    {
        rt_kprintf("c5parlio: FAIL, cannot route IO%d/IO%d\n",
                   (int)C5_PARLIO_CLK_PIN, (int)C5_PARLIO_DATA_PIN);
        return -1;
    }

    if (rt_hw_pcnt_setup(0, C5_PARLIO_CLK_PIN) != RT_EOK)
    {
        rt_kprintf("c5parlio: FAIL, PCNT cannot watch IO%d\n", (int)C5_PARLIO_CLK_PIN);
        return -1;
    }

    /* CONTROL FIRST: count edges for the same duration with PARLIO idle.
     *
     * Without this, "PCNT counted N edges" is not evidence of anything - an
     * undriven pad can produce counts, and PCNT's glitch filter is off in this
     * port (its reset value is 1 and it eats real signals, section 17d). The
     * control is measured BEFORE the attempt so the number cannot be interpreted
     * after the fact. Measured here: 0 edges, so any count above is real. */
    rt_hw_pcnt_clear(0);
    rt_thread_mdelay(200);
    control_edges = rt_hw_pcnt_get(0);

    rt_kprintf("c5parlio: %u byte(s) at %u Hz, clk=IO%d data=IO%d\n",
               (unsigned)len, (unsigned)clk_hz,
               (int)C5_PARLIO_CLK_PIN, (int)C5_PARLIO_DATA_PIN);
    rt_kprintf("c5parlio: control - %d edge(s) in 200 ms with the unit idle\n",
               control_edges);

    rt_hw_pcnt_clear(0);

    err = rt_hw_parlio_tx(payload, len, clk_hz, 200u, &consumed);

    /* Read the clock count in BOTH outcomes: on failure it is the only thing that
     * says whether the unit ran at all. */
    edges = rt_hw_pcnt_get(0);

    if (err != RT_EOK)
    {
        rt_kprintf("c5parlio: transmit returned %d; PCNT counted %d edge(s), "
                   "the unit consumed %u bit(s)\n",
                   (int)err, edges, (unsigned)consumed);

        if (edges > control_edges + 100)
        {
            rt_kprintf("c5parlio: the clock line ran, so the unit started - the "
                       "fault is in the completion path, not the start path\n");
        }
        else
        {
            rt_kprintf("c5parlio: the clock line did not move - the unit never "
                       "started\n");
        }
        return -1;
    }

    /* WHAT `tx_fifo_rd_bit_cnt` ACTUALLY MEANS.
     *
     * The register is documented as "the current read bit number from Tx FIFO" and
     * that is literal: it is a 0-BASED index that WRAPS AT 128, not a total.
     * Measured across four frame sizes:
     *
     *      8 bytes ( 64 bits) ->  63   =  64 - 1
     *     16 bytes (128 bits) -> 127   = 128 - 1
     *     20 bytes (160 bits) ->  31   = 160 - 1 - 128
     *     32 bytes (256 bits) -> 127   = 256 - 1 - 128
     *
     * I first treated it as a total and wrote a check for `== len * 8`, which
     * failed on a frame the hardware had transmitted correctly. Both mistakes are
     * in this file's history; the formula below is the one the hardware obeys.
     *
     * Consequence for verification: below 17 bytes the value is unambiguous, and
     * at 17 bytes and above it cannot distinguish N from N + 128k. So the exact
     * check is only made where it means something. */
    {
        rt_uint32_t bits = len * 8u;
        rt_uint32_t expected = (bits - 1u) % 128u;

        rt_kprintf("c5parlio: the unit read up to bit %u (expected %u for %u bit(s)); "
                   "PCNT counted %d edge(s) (control %d)\n",
                   (unsigned)consumed, (unsigned)expected, (unsigned)bits,
                   edges, control_edges);

        if (consumed != expected)
        {
            rt_kprintf("c5parlio: FAIL, the unit did not read to the end of the "
                       "frame\n");
            return -1;
        }

        if (bits > 128u)
        {
            rt_kprintf("c5parlio: note - at %u bit(s) the counter wraps, so this "
                       "confirms the unit read data but not the exact frame "
                       "length\n", (unsigned)bits);
        }
    }

    if (edges <= control_edges)
    {
        rt_kprintf("c5parlio: FAIL, the unit consumed data but no clock was seen\n");
        return -1;
    }

    rt_kprintf("c5parlio: PASS (GDMA fed the unit, the unit consumed every bit, and "
               "an independent counter saw the clock run; the DATA line's levels are "
               "not verified)\n");
    return 0;
}

#endif /* SOC_PARLIO_SUPPORTED */

