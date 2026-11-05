/*
 * ESP32-C5 RMT (Remote Control Transceiver): TX, as a plain C API.
 *
 * RT-Thread has no RMT class, so like PCNT and MCPWM this is an API plus an msh
 * command rather than an rt_device.
 *
 * ---------------------------------------------------------------------------
 * How this is verified with no external instrument.
 *
 * A transmitted waveform is only "correct" if something counts its edges. The
 * RMT's own RX channel can do that: channel 0 transmits a square wave and
 * channel 2 (the C5's first RX channel) receives it on the same pad, because
 * RMT_SIG_OUT0 and RMT_SIG_IN0 are separate matrix indices that can be pointed
 * at one pin. The received symbols then give the pulse count and widths, which
 * is a real measurement of what left the transmitter rather than a read-back of
 * the registers that were written.
 *
 * What is NOT verified and is stated as such: the analog shape of the pulses,
 * and anything about the pad drivers themselves - the loop is inside the GPIO
 * matrix, so nothing leaves the chip.
 *
 * ---------------------------------------------------------------------------
 * C5 specifics, versus the S31 file this was ported from:
 *
 *  - TWO TX and TWO RX channels, not four and four. `soc_rmt_signals[0]` gives
 *    ch0/ch1 as TX (RMT_SIG_OUT0/1) and ch2/ch3 as RX (RMT_SIG_IN0/1), with the
 *    TX entries carrying `rx_sig = -1` and vice versa. A driver that assumes
 *    every channel can do both writes a channel that silently does nothing.
 *
 *  - The symbol memory is at RMTMEM (0x60006400, from esp32c5.peripherals.ld),
 *    48 words per channel. IDF writes it directly rather than through the APB
 *    FIFO, which avoids all FIFO bookkeeping. The struct is re-declared here so
 *    this file does not depend on esp_driver_rmt, which this port does not link.
 *
 *  - `sys_conf.apb_fifo_mask` must be set to 1 before addressing the array:
 *    with the reset value (0) an APB store into the RMTMEM window pushes the
 *    FIFO instead, every symbol reads back as zero, and the transmitter keeps
 *    sending whatever the array already held. IDF sets the same bit in
 *    rmt_hal_init().
 *
 *  - The memory array has its own low-power domain and defaults to low power.
 *    `rmt_ll_mem_force_power_on()` is required; the bus clock alone does not
 *    power it, and without it every write lands in a dead array.
 *
 *  - The carrier generator is ON out of reset (`chnconf1.carrier_en_chn` reads
 *    back 1). Left on, every symbol is modulated by the carrier: the frame's
 *    total duration is right but the transitions per symbol jump from 1 to ~32,
 *    which looks like a timing bug and is not one.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_struct.h"
#include "hal/gpio_ll.h"
#include "hal/rmt_ll.h"
#include "hal/rmt_types.h"
#include "esp_rom_gpio.h"
#include "esp_private/esp_clk.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_RMT_SUPPORTED)

#include "port_rmt.h"

/* Shape of RMTMEM, from the target's peripherals linker script. Declared here
 * rather than including esp_driver_rmt's private header. */
typedef struct
{
    struct
    {
        rmt_symbol_word_t symbols[SOC_RMT_MEM_WORDS_PER_CHANNEL];
    } channels[8];
} c5_rmt_block_mem_t;

extern c5_rmt_block_mem_t RMTMEM;

/* XTAL 48 MHz, divider 1 -> ~20.8 ns per RMT tick. */
static rt_uint32_t s_rmt_tick_ns = 21u;

static rmt_dev_t *s_rmt;

static inline rt_uint32_t c5_rmt_cycles(void)
{
    rt_uint32_t v;

    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));

    return v;
}

static inline rt_bool_t c5_rmt_cycles_exceeded(rt_uint32_t start, rt_uint32_t ms)
{
    return (rt_uint32_t)(c5_rmt_cycles() - start) >
           (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000u * ms);
}

rt_err_t rt_hw_rmt_init(void)
{
    PERIPH_RCC_ATOMIC()
    {
        rmt_ll_enable_bus_clock(C5_RMT_GROUP, true);
        rmt_ll_reset_register(C5_RMT_GROUP);
    }

    s_rmt = &RMT;

    /* Power the symbol memory. Without this every write to RMTMEM reads back as
     * zero and nothing is transmitted - the same failure the S31 file records. */
    rmt_ll_mem_force_power_on(s_rmt);

    /* Address the memory array, not the FIFO. With the reset value an APB store
     * into the RMTMEM window pushes the FIFO, every symbol reads back zero, and
     * the transmitter keeps sending whatever the array already held. */
    rmt_ll_enable_mem_access_nonfifo(s_rmt, true);

    /* XTAL, not the default PLL_F80M - measured three times in this port now
     * (鎼?7 ADC, 鎼?7b CAN): the PLL is not enabled in this image unless
     * something asks for it, so a block whose clock source defaults to PLL gets
     * no clock at all. XTAL is also the stable choice for a pulse generator. */
    rmt_ll_set_group_clock_src(s_rmt, C5_RMT_TX_CH, RMT_CLK_SRC_XTAL, 1, 1, 0);
    rmt_ll_enable_group_clock(s_rmt, true);

    /* Divider 1 gives exactly the source rate, so one tick is 1/xtal_hz seconds.
     *
     * Round to the nearest nanosecond: 1e9/48e6 = 20.83 -> 21. The first version
     * of this line computed 1 ns, which made every requested duration 20x too
     * long and is what the "32767 tick(s)/half = 100 us, tick=1 ns" line in the
     * first run reported.
     *
     * The remainder is kept as a percentage rather than thrown away: ticks are
     * integers, so durations below ~21 ns cannot be represented at all and
     * anything above is quantised to 21 ns steps. */
    {
        rt_uint32_t xtal = (rt_uint32_t)esp_clk_xtal_freq();

        s_rmt_tick_ns = (1000000000u + xtal / 2u) / xtal;
        if (s_rmt_tick_ns == 0u)
        {
            s_rmt_tick_ns = 1u;
        }
    }

    rmt_ll_tx_set_channel_clock_div(s_rmt, C5_RMT_TX_CH, 1);
    rmt_ll_tx_set_mem_blocks(s_rmt, C5_RMT_TX_CH, 1);
    rmt_ll_tx_reset_pointer(s_rmt, C5_RMT_TX_CH);

    /* Turn the carrier OFF. Its reset value is ON, and left on every symbol is
     * modulated by it: correct frame duration, ~32 transitions per symbol
     * instead of 1. A default-on feature nobody asked for. */
    rmt_ll_tx_enable_carrier_modulation(s_rmt, C5_RMT_TX_CH, false);

    /* Idle level low, driven. A channel that is not driving leaves the pad
     * floating between transmissions. */
    rmt_ll_tx_fix_idle_level(s_rmt, C5_RMT_TX_CH, 0, true);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_rmt_init);

rt_err_t rt_hw_rmt_set_pin(rt_base_t pin)
{
    if (pin < 0 || pin >= SOC_GPIO_PIN_COUNT ||
        ((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
    gpio_ll_output_enable(&GPIO, (uint8_t)pin);

    /* ctrl_by_periph = true, oen_inv = FALSE.
     *
     * The second argument is the INVERT flag, not "enable". Passing true there
     * inverts the output enable, so the pad is disabled exactly when the RMT
     * wants to drive it: the transmitter runs, ch0_tx_end fires, and nothing
     * ever appears on the pad or in the loopback. Measured: txdone=1,
     * int_raw=0x1, and rx still captured zero symbols.
     *
     * The CAN driver gets this right by accident of being written later
     * (twai.c passes false) - the same mistake there would have looked like a
     * broken transceiver. */
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin, true, false);
    gpio_ll_input_enable(&GPIO, (uint8_t)pin);
    gpio_ll_pullup_dis(&GPIO, (uint8_t)pin);
    gpio_ll_pulldown_dis(&GPIO, (uint8_t)pin);

    esp_rom_gpio_connect_out_signal((uint32_t)pin, RMT_SIG_OUT0_IDX, false, false);

    /* Loop the pad's own input back into the RX channel's input signal. This is
     * what lets one pad be both the generator and the observer: RMT_SIG_OUT0
     * and RMT_SIG_IN0 are distinct matrix indices, so the pad can drive one and
     * feed the other. It is the same trick the CAN driver uses to loop TX into
     * RX on a single pad. */
    esp_rom_gpio_connect_in_signal((uint32_t)pin, RMT_SIG_IN0_IDX, false);

    return RT_EOK;
}

/* One symbol: level0 for d0 ticks, then level1 for d1 ticks. */
static void c5_rmt_put(rt_uint32_t idx, rt_uint32_t d0, rt_uint32_t l0,
                       rt_uint32_t d1, rt_uint32_t l1)
{
    rmt_symbol_word_t s;

    if (d0 > 0x7FFFu) d0 = 0x7FFFu;
    if (d1 > 0x7FFFu) d1 = 0x7FFFu;

    s.duration0 = d0 & 0x7FFFu;
    s.level0    = l0 & 1u;
    s.duration1 = d1 & 0x7FFFu;
    s.level1    = l1 & 1u;

    RMTMEM.channels[C5_RMT_TX_CH].symbols[idx].val = s.val;
}

/* Transmit `symbols` symbols from channel 0 and wait for the frame to drain. */
rt_err_t rt_hw_rmt_tx(rt_uint32_t symbols, rt_uint32_t timeout_ms)
{
    rt_uint32_t start;

    if (s_rmt == RT_NULL || symbols == 0u || symbols > SOC_RMT_MEM_WORDS_PER_CHANNEL)
    {
        return -RT_EINVAL;
    }

    rmt_ll_tx_stop(s_rmt, C5_RMT_TX_CH);
    rmt_ll_tx_reset_pointer(s_rmt, C5_RMT_TX_CH);
    /* tx_lim is an interrupt threshold, not an end marker, so it is set to the
     * block size: the frame ends at the stop pattern, not here. */
    rmt_ll_tx_set_limit(s_rmt, C5_RMT_TX_CH, SOC_RMT_MEM_WORDS_PER_CHANNEL);
    rmt_ll_clear_interrupt_status(s_rmt, UINT32_MAX);

    rmt_ll_tx_start(s_rmt, C5_RMT_TX_CH);

    /* Bounded wait. The idle flag is the only completion signal the LL exposes
     * on this chip - rmt_ll_tx_is_channel_idle() does not exist here (the S31
     * has it), so the status flag is read directly. Leaving this unbounded
     * would hang the shell on a channel that never starts. */
    /* Completion is the ch0_tx_end interrupt, NOT a state poll.
     *
     * state_chn is a three-bit field (rmt_struct.h: bitpos [11:9]) that runs
     * 0 (idle) -> 1 (start) -> 2 (running) -> 0 (idle). Polling it for "returns
     * to idle" looks correct and is not: a frame that completes faster than the
     * first read leaves the field at 0 the whole time, so the loop never sees it
     * leave and burns the entire timeout on a transmission that already
     * finished. Measured exactly that: the first version reported
     * -RT_ETIMEOUT while the TX-done evidence below says the frame went out.
     *
     * ch0_tx_end_int_raw (int_raw bit 0) is latched by hardware when the frame
     * drains, so it cannot be missed. */
    start = c5_rmt_cycles();
    while (!s_rmt->int_raw.ch0_tx_end_int_raw)
    {
        if (c5_rmt_cycles_exceeded(start, timeout_ms))
        {
            rmt_ll_tx_stop(s_rmt, C5_RMT_TX_CH);
            return -RT_ETIMEOUT;
        }
    }

    return RT_EOK;
}

rt_uint32_t rt_hw_rmt_tick_ns(void)
{
    return s_rmt_tick_ns;
}

/* ------------------------------------------------------------------ msh -- */

/* `c5rmt [half_period_us]` - transmit a square wave and count what came back.
 *
 * Channel 0 drives the pad, the pad's own input feeds RX channel 0's input
 * signal, and the received symbols are read out of RMTMEM. The pulse count and
 * the measured half period are then compared with what was asked for - a real
 * measurement of the transmitted waveform rather than a register read-back.
 */
int c5_rmt_selftest(rt_uint32_t half_us)
{
    rt_uint32_t ticks;
    rt_uint32_t sym;
    rt_uint32_t pulses = 0;
    rt_uint32_t received = 0;
    rt_uint32_t sum_ticks = 0;
    rt_err_t err;

    if (s_rmt == RT_NULL)
    {
        rt_kprintf("c5rmt   : FAIL, RMT not initialised\n");
        return -1;
    }

    if (half_us == 0u || half_us > 1000u)
    {
        half_us = 100u;         /* 5 kHz square wave */
    }

    /* First, prove the pad and the matrix input path work at all, with no RMT
     * involved: drive the pad from the plain GPIO output register and read its
     * own input back.
     *
     * This has to happen BEFORE rt_hw_rmt_set_pin(), because that call points the
     * pad's output multiplexer at RMT_SIG_OUT0 - after which GPIO_ENABLE and the
     * GPIO output register no longer reach the pin at all, and an identical test
     * reports "PAD DOES NOT FOLLOW" on a perfectly good pad. Measured that
     * ordering mistake first. */
    {
        rt_uint32_t hi;
        rt_uint32_t lo;

        gpio_ll_func_sel(&GPIO, (uint8_t)C5_RMT_TEST_PIN, PIN_FUNC_GPIO);
        esp_rom_gpio_connect_out_signal((uint32_t)C5_RMT_TEST_PIN,
                                        SIG_GPIO_OUT_IDX, false, false);
        gpio_ll_output_enable(&GPIO, (uint8_t)C5_RMT_TEST_PIN);
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)C5_RMT_TEST_PIN, false, false);
        gpio_ll_input_enable(&GPIO, (uint8_t)C5_RMT_TEST_PIN);

        gpio_ll_set_level(&GPIO, (uint8_t)C5_RMT_TEST_PIN, 1);
        hi = gpio_ll_get_level(&GPIO, (uint8_t)C5_RMT_TEST_PIN);

        gpio_ll_set_level(&GPIO, (uint8_t)C5_RMT_TEST_PIN, 0);
        lo = gpio_ll_get_level(&GPIO, (uint8_t)C5_RMT_TEST_PIN);

        rt_kprintf("c5rmt   : pad self-check IO%d: drove 1 -> read %u, drove 0 -> read %u (%s)\n",
                   (int)C5_RMT_TEST_PIN, (unsigned)hi, (unsigned)lo,
                   (hi == 1u && lo == 0u) ? "pad and input path OK"
                                          : "PAD DOES NOT FOLLOW - loopback impossible");
    }

    rt_hw_rmt_set_pin(C5_RMT_TEST_PIN);

    /* Ticks per half period, from the measured tick length rather than from an
     * assumed clock. */
    ticks = (half_us * 1000u) / s_rmt_tick_ns;
    if (ticks == 0u)
    {
        ticks = 1u;
    }
    if (ticks > 0x7FFFu)
    {
        ticks = 0x7FFFu;
    }

    /* 20 symbols of square wave, then the END MARKER.
     *
     * An RMT transmission does not end because the memory block ran out - it
     * ends at a symbol whose duration0 is zero. IDF says so outright
     * (rmt_tx.c:612): "a RMT word whose duration is zero means a stop pattern",
     * written as { duration0 = 0, duration1 = 0, level = eot_level }.
     *
     * Measured without it: a full block of real symbols transmitted happily but
     * never raised ch0_tx_end (int_raw bit 0); the only status bit set was
     * ch0_tx_thr_event (bit 8), which is the "sent more than tx_lim" threshold
     * and not an end-of-frame signal at all. */
    for (sym = 0; sym < 20u; sym++)
    {
        if ((sym & 1u) == 0u)
        {
            c5_rmt_put(sym, ticks, 1, ticks, 0);
        }
        else
        {
            c5_rmt_put(sym, ticks, 0, ticks, 1);
        }
    }

    /* The stop pattern. Levels match the idle level so the pad does not glitch
     * when the frame ends. */
    c5_rmt_put(20u, 0u, 0u, 0u, 0u);


    /* RX channel 2 receives on the same pad, one memory block. */
    /* The memory block has to be handed to the RX engine explicitly
     * (chmconf.conf1.mem_owner_chm). Without this the RX channel is enabled,
     * the transmitter runs, and RMTMEM still reads all zeros. */
    rmt_ll_rx_set_mem_owner(s_rmt, C5_RMT_RX_CH, RMT_LL_MEM_OWNER_HW);
    rmt_ll_rx_set_channel_clock_div(s_rmt, C5_RMT_RX_CH, 1);
    rmt_ll_rx_set_mem_blocks(s_rmt, C5_RMT_RX_CH, 1);
    rmt_ll_rx_reset_pointer(s_rmt, C5_RMT_RX_CH);
    /* Idle threshold at the maximum, not a multiple of the half period.
     *
     * The receiver ends a frame after `idle_thres` ticks with no edge. Any value
     * derived from the half period is fine in principle, but the gap between
     * enabling RX and starting TX is not bounded by that period - and a frame
     * that ends during that gap captures one idle symbol and nothing else, which
     * is exactly what the first version measured (write pointer advanced by 1).
     * The maximum makes the pre-transmission gap irrelevant; the frame then ends
     * on the transmitter's stop pattern instead. */
    rmt_ll_rx_set_idle_thres(s_rmt, C5_RMT_RX_CH, 0x7FFFu);
    rmt_ll_rx_set_limit(s_rmt, C5_RMT_RX_CH, SOC_RMT_MEM_WORDS_PER_CHANNEL);
    rmt_ll_rx_enable_filter(s_rmt, C5_RMT_RX_CH, false);
    rmt_ll_rx_enable(s_rmt, C5_RMT_RX_CH, true);

    err = rt_hw_rmt_tx(20u, 50u);

    rmt_ll_rx_enable(s_rmt, C5_RMT_RX_CH, false);

    /* Did the transmitter actually run?
     *
     * A frame that never leaves TX looks exactly like a loop that is not wired:
     * both give zero received symbols. Reading the TX channels read pointer and
     * the TX-done interrupt bit separates them - if the pointer moved and
     * TX_DONE fired, the pulses went out and the problem is on the RX side. */
    rt_kprintf("c5rmt   : tx status: state=%u raddr=%u txdone=%u intraw=0x%08x\n",
               (unsigned)s_rmt->chnstatus[C5_RMT_TX_CH].state_chn,
               (unsigned)s_rmt->chnstatus[C5_RMT_TX_CH].mem_raddr_ex_chn,
               (unsigned)((s_rmt->int_raw.val >> 0) & 1u),
               (unsigned)s_rmt->int_raw.val);
    rt_kprintf("c5rmt   : rx status: state=%u waddr=%u rxdone=%u\n",
               (unsigned)s_rmt->chmstatus[C5_RMT_RX_CH].mem_waddr_ex_chm,
               (unsigned)s_rmt->chmstatus[C5_RMT_RX_CH].apb_mem_raddr_chm,
               (unsigned)((s_rmt->int_raw.val >> 2) & 1u));

    if (err != RT_EOK)
    {
        rt_kprintf("c5rmt   : FAIL, transmit did not complete (%d)\n", (int)err);
        return -1;
    }

    /* Hand the memory back to software BEFORE reading it.
     *
     * While `mem_owner_chm` is HW the RX engine owns the block and APB reads
     * return zero - so a receiver that captured everything reads as "captured
     * nothing". IDF does exactly this handoff around its memcpy
     * (rmt_rx.c:655-658): SW, copy, then HW again.
     *
     * Measured before this fix: int_raw had both ch0_tx_end (bit 0) and
     * ch2_rx_end (bit 2) set and the write pointer stood at 21 symbols, yet
     * every read returned 0. */
    rmt_ll_rx_set_mem_owner(s_rmt, C5_RMT_RX_CH, RMT_LL_MEM_OWNER_SW);

    /* Reference point: the symbols the TX channel was given, read back from the
     * APB window. If these are non-zero then the window works and the RX block
     * really is empty; if they are zero too, the window itself is not readable
     * and nothing about RX can be concluded from it. */
    rt_kprintf("c5rmt   : mem probe tx[0]=0x%08x tx[1]=0x%08x rx[0]=0x%08x rx[1]=0x%08x\n",
               (unsigned)RMTMEM.channels[C5_RMT_TX_CH].symbols[0].val,
               (unsigned)RMTMEM.channels[C5_RMT_TX_CH].symbols[1].val,
               (unsigned)RMTMEM.channels[C5_RMT_RX_MEM_CH].symbols[0].val,
               (unsigned)RMTMEM.channels[C5_RMT_RX_MEM_CH].symbols[1].val);
    rt_kprintf("c5rmt   : rx waddr(chm)=%u apb_raddr=%u\n",
               (unsigned)s_rmt->chmstatus[C5_RMT_RX_CH].mem_waddr_ex_chm,
               (unsigned)s_rmt->chmstatus[C5_RMT_RX_CH].apb_mem_raddr_chm);

    /* Count what the RX channel captured. */
    for (sym = 0; sym < SOC_RMT_MEM_WORDS_PER_CHANNEL; sym++)
    {
        rmt_symbol_word_t s;

        s.val = RMTMEM.channels[C5_RMT_RX_MEM_CH].symbols[sym].val;

        if (s.val == 0u)
        {
            break;
        }

        received++;
        if (s.duration0)
        {
            pulses++;
            sum_ticks += s.duration0;
        }
        if (s.duration1)
        {
            pulses++;
            sum_ticks += s.duration1;
        }
    }

    /* Decisive test: does the pad actually toggle while RMT transmits?
     *
     * Everything else about this driver has been checked and is correct - the
     * symbols in RMTMEM decode to exactly the requested waveform, tx_end fires,
     * the pad drives correctly under plain GPIO control. What is not known is
     * whether RMT_SIG_OUT0 reaches the pin at all.
     *
     * So: put the channel in continuous mode, sample the pad's own input through
     * the GPIO register a few thousand times, and count the transitions. A
     * toggling pad means the transmitter reaches the pin and the fault is on the
     * RX side; a static pad means the output routing is wrong. */
    {
        rt_uint32_t n;
        rt_uint32_t transitions = 0;
        rt_uint32_t prev;

        rmt_ll_tx_enable_loop(s_rmt, C5_RMT_TX_CH, true);
        rmt_ll_clear_interrupt_status(s_rmt, UINT32_MAX);
        rmt_ll_tx_reset_pointer(s_rmt, C5_RMT_TX_CH);
        rmt_ll_tx_start(s_rmt, C5_RMT_TX_CH);

        prev = gpio_ll_get_level(&GPIO, (uint8_t)C5_RMT_TEST_PIN);
        for (n = 0; n < 20000u; n++)
        {
            rt_uint32_t now = gpio_ll_get_level(&GPIO, (uint8_t)C5_RMT_TEST_PIN);

            if (now != prev)
            {
                transitions++;
                prev = now;
            }
        }

        rmt_ll_tx_stop(s_rmt, C5_RMT_TX_CH);
        rmt_ll_tx_enable_loop(s_rmt, C5_RMT_TX_CH, false);
        rmt_ll_clear_interrupt_status(s_rmt, UINT32_MAX);

        rt_kprintf("c5rmt   : pad sampling during continuous TX: %u transition(s) "
                   "in 20000 reads -> %s\n",
                   (unsigned)transitions,
                   transitions ? "RMT reaches the pad, fault is on the RX side"
                               : "RMT does NOT reach the pad");
    }
    rt_kprintf("c5rmt   : tx %u symbols, %u tick(s)/half = %u us, tick=%u ns\n",
               (unsigned)SOC_RMT_MEM_WORDS_PER_CHANNEL,
               (unsigned)ticks, (unsigned)half_us, (unsigned)s_rmt_tick_ns);
    rt_kprintf("c5rmt   : rx %u symbol(s), %u pulse(s), mean half = %u tick(s)\n",
               (unsigned)received, (unsigned)pulses,
               (unsigned)(pulses ? sum_ticks / pulses : 0u));

    if (received == 0u)
    {
        rt_kprintf("c5rmt   : FAIL, nothing was received back - the loop or the "
                   "transmitter is not working\n");
        return -1;
    }

    /* The mean half period has to be within 20 % of the request. A wrong
     * divider or a wrong clock source shows up here as a ratio, not as an
     * absolute error, so the bound is relative. */
    {
        rt_uint32_t mean = sum_ticks / (pulses ? pulses : 1u);
        rt_uint32_t lo = ticks - (ticks / 5u);
        rt_uint32_t hi = ticks + (ticks / 5u);

        if (mean < lo || mean > hi)
        {
            rt_kprintf("c5rmt   : FAIL, mean half period %u tick(s) is not within "
                       "20%% of %u\n", (unsigned)mean, (unsigned)ticks);
            return -1;
        }
    }

    rt_kprintf("c5rmt   : PASS (transmitted waveform measured by the RX channel; "
               "the pad driver itself is inside the matrix and not verified)\n");
    return 0;
}

#endif /* SOC_RMT_SUPPORTED */




























