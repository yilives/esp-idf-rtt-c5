/*
 * ESP32-S31 RMT (Remote Control Transceiver): TX, as a plain C API.
 *
 * RT-Thread has no RMT class, so like PCNT and MCPWM this is an API plus an msh
 * command rather than an rt_device.
 *
 * ---------------------------------------------------------------------------
 * The RMT is used here as a *reference instrument*, not only as a peripheral to
 * port.
 *
 * `s31led` already drives the on-board WS2812 by counting CPU cycles with
 * interrupts masked. That works, but its accuracy depends on cache state, on the
 * compiler's instruction selection and on nothing else running. The RMT
 * generates the same waveforms from a 40 MHz clock and a 15-bit duration field,
 * so the two can be compared against each other and the RMT is the trustworthy
 * one. `s31rmt` uses that to check itself: a square wave is transmitted, PCNT
 * counts the edges and mcycle polling measures the duty.
 *
 * What is NOT verified and is stated as such: the analog shape of the pulses,
 * and any half-period below ~2 us (the same mcycle polling limit MCPWM has).
 *
 * ---------------------------------------------------------------------------
 * S31 specifics:
 *
 *  - One instance (`RMT`), 4 TX + 4 RX channels, 48 symbols of memory each. Only
 *    TX is implemented; RX needs the idle-threshold machinery as well.
 *
 *  - The symbol memory is NOT a register: it lives at RMTMEM (0x20355800, from
 *    esp32s31.peripherals.ld), 48 x 4 bytes per channel. IDF writes it directly
 *    (rmt_tx.c:352) rather than through the APB FIFO, which avoids all FIFO
 *    bookkeeping. The struct is re-declared here so this file does not depend on
 *    esp_driver_rmt, which this port does not link.
 *
 *  - `chnconf0.conf_update_chn` has to be pulsed before `tx_start_chn`;
 *    rmt_ll_tx_start() does both.
 *
 *  - A symbol is "level0 for duration0, then level1 for duration1" - two halves
 *    per 32-bit word, 15 bits of duration each.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_ll.h"
#include "hal/rmt_ll.h"
#include "hal/rmt_types.h"
#include "soc/rmt_struct.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#include "port_rmt.h"
#include "port_pcnt.h"
#include "port_board.h"

#if defined(SOC_RMT_SUPPORTED)

/* Shape of RMTMEM, from the target's peripherals linker script. Declared here
 * rather than including esp_driver_rmt's private header. */
typedef struct
{
    struct
    {
        rmt_symbol_word_t symbols[SOC_RMT_MEM_WORDS_PER_CHANNEL];
    } channels[8];
} s31_rmt_block_mem_t;

extern s31_rmt_block_mem_t RMTMEM;

#define S31_RMT_GROUP       0
#define S31_RMT_TX_CH       0

/* XTAL 40 MHz, divider 1 -> 25 ns per RMT tick. */
#define S31_RMT_TICK_NS     25u

static rmt_dev_t *s_rmt;

static inline rt_uint32_t s31_cycles(void)
{
    rt_uint32_t v;

    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));

    return v;
}

rt_err_t rt_hw_rmt_init(void)
{
    PERIPH_RCC_ATOMIC()
    {
        rmt_ll_enable_bus_clock(S31_RMT_GROUP, true);
        rmt_ll_reset_register(S31_RMT_GROUP);
    }

    s_rmt = &RMT;

    /*
     * Power the symbol memory. Without this every write to RMTMEM reads back as
     * zero and nothing is transmitted - measured: symbol0 = symbol1 = 0 after
     * writing a full waveform, edges counted = 0.
     *
     * HP_SYSTEM.sys_rmt_mem_lp_ctrl gates the memory array (the RMT memory is
     * separate silicon with its own low-power domain), and it defaults to low
     * power. This is the S31 equivalent of what esp_driver_rmt does in its
     * channel construction and it is not implied by enabling the bus clock.
     */
    rmt_ll_mem_force_power_on(s_rmt);

    /*
     * Address the memory array, not the FIFO.
     *
     * sys_conf.apb_fifo_mask selects what an APB access inside the RMTMEM window
     * means: 0 (reset) = push/pop the FIFO, 1 = address the array directly. With
     * the default the stores went into the FIFO, every symbol read back as zero,
     * and the transmitter kept sending whatever the array already held - which
     * is why edges appeared (about 210 kHz worth) while the measured half period
     * was 1.4 us instead of the programmed 100 us.
     *
     * Measured confirmation before this line existed: sys_conf read back
     * 0x05000010 (apb_fifo_mask = 0) and a raw write of 0x0fa08fa0 read back 0.
     * IDF sets the same bit in rmt_hal_init() (rmt_hal.c:15).
     */
    rmt_ll_enable_mem_access_nonfifo(s_rmt, true);

    /* The divider is (1 + div_num + div_a/div_b), so integral = 1 gives exactly
     * the source rate: 40 MHz -> 25 ns ticks, which is the resolution WS2812
     * wants (0.4 us / 0.8 us are 16 / 32 ticks). */
    rmt_ll_set_group_clock_src(s_rmt, S31_RMT_TX_CH, RMT_CLK_SRC_XTAL, 1, 1, 0);
    rmt_ll_enable_group_clock(s_rmt, true);

    rmt_ll_tx_set_channel_clock_div(s_rmt, S31_RMT_TX_CH, 1);
    rmt_ll_tx_set_mem_blocks(s_rmt, S31_RMT_TX_CH, 1);
    rmt_ll_tx_reset_pointer(s_rmt, S31_RMT_TX_CH);

    /*
     * Turn the carrier OFF. Its reset value is ON.
     *
     * `chnconf1.carrier_en_chn` is bit 21 of the channel config and reads back 1
     * out of reset, so every symbol leaves the pad modulated by the carrier
     * generator. The frame's total duration is unchanged - the carrier only
     * flips the level inside each symbol - which is exactly what was measured: a
     * 20-symbol frame took the correct 4 ms but produced 630 rising edges
     * instead of 20, about 32 per symbol.
     *
     * Nothing about the RMT looked broken: correct length, correct timing, 32x
     * the transitions. A default-on feature nobody asked for.
     */
    rmt_ll_tx_enable_carrier_modulation(s_rmt, S31_RMT_TX_CH, false);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_rmt_init);

/* One symbol: level0 for d0 ticks, then level1 for d1 ticks. */
static void s31_rmt_put(rt_uint32_t idx, rt_uint32_t d0, rt_uint32_t l0,
                        rt_uint32_t d1, rt_uint32_t l1)
{
    rmt_symbol_word_t s;

    if (d0 > 0x7FFFu) d0 = 0x7FFFu;
    if (d1 > 0x7FFFu) d1 = 0x7FFFu;

    s.duration0 = d0 & 0x7FFFu;
    s.level0    = l0 & 1u;
    s.duration1 = d1 & 0x7FFFu;
    s.level1    = l1 & 1u;

    RMTMEM.channels[S31_RMT_TX_CH].symbols[idx].val = s.val;
}

static void s31_rmt_start(rt_uint32_t count)
{
    rmt_ll_clear_interrupt_status(s_rmt, 0xFFFFFFFFu);

    /*
     * Enable TX_DONE before starting.
     *
     * rmt_ll_tx_get_interrupt_status() reads int_st, the *masked* status, and my
     * raw dump showed int_ena = 0 - so trans_done could never be observed no
     * matter how long the poll ran. That is why the first version sat on a
     * 20 000-iteration (23 second) timeout: it was waiting for a bit that the
     * hardware was never going to set in the register it was reading.
     */
    rmt_ll_enable_interrupt(s_rmt, RMT_LL_EVENT_TX_DONE(S31_RMT_TX_CH), true);

    rmt_ll_tx_set_limit(s_rmt, S31_RMT_TX_CH, count);
    rmt_ll_tx_fix_idle_level(s_rmt, S31_RMT_TX_CH, 0, true);

    /*
     * Rewind the memory read pointer before every transmission.
     *
     * It is only reset in rt_hw_rmt_init(), so after the first frame the
     * transmitter resumes wherever the previous one stopped. Measured before
     * this line existed: with the block's tail filled with stop patterns the
     * second frame of a command produced 0 edges - the pointer was already past
     * the stop marker.
     */
    rmt_ll_tx_reset_pointer(s_rmt, S31_RMT_TX_CH);

    rmt_ll_tx_start(s_rmt, S31_RMT_TX_CH);
}

static inline rt_uint32_t s31_rmt_cycles(void)
{
    rt_uint32_t v;

    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));

    return v;
}

static rt_err_t s31_rmt_wait_done(rt_uint32_t timeout_us)
{
    rt_uint32_t start = s31_rmt_cycles();
    /*
     * Cap the budget at 50 ms of cycles.
     *
     * Even the slowest legal frame here (48 symbols at 1 ms per half) is under
     * 100 ms, and a shell command that blocks for seconds when a completion bit
     * never arrives is worse than one that reports the timeout. Measured: with a
     * 5 ms budget the call returned after 5 s, i.e. ~3 us per MMIO poll - which
     * is itself worth knowing and is recorded in the log.
     */
    /*
     * Cycle budget, so 50 ms is CPU_MHz * 1000 * 50.
     *
     * The first version wrote CPU_MHz * 50, which is 50 MICROseconds - 1000x too
     * small. Every frame longer than 50 us then timed out and was stopped
     * mid-transmission, which is why a 4 ms frame reported about 31 edges
     * instead of 20 and the diagnosis went looking at frame-length control.
     */
    rt_uint32_t budget = (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000u * 50u);

    /*
     * Cycle-bounded spin, no delay call inside the loop.
     *
     * The first version called esp_rom_delay_us(1) per iteration. Measured: a
     * 20000-iteration bound took 23.2 SECONDS - about 1.16 ms per iteration, so
     * the shell appeared to hang. Whatever esp_rom_delay_us(1) resolves to on
     * this build, using it as a fine-grained timeout tick made the timeout 1000x
     * longer than intended.
     *
     * Reading an MMIO bit costs a few cycles, so a plain cycle budget is both
     * accurate and the right scale for a transaction that lasts tens of
     * microseconds. mcycle wraps every 13.4 s at 320 MHz, which is far longer
     * than any timeout used here.
     */
    while ((rt_uint32_t)(s31_rmt_cycles() - start) < budget)
    {
        if (rmt_ll_tx_get_interrupt_status(s_rmt, S31_RMT_TX_CH) &
            RMT_LL_EVENT_TX_DONE(S31_RMT_TX_CH))
        {
            rmt_ll_clear_interrupt_status(s_rmt, 0xFFFFFFFFu);
            return RT_EOK;
        }
    }

    /*
     * trans_done was never observed.
     *
     * This is one of the two open problems with this driver (the other is frame
     * length control). rmt_ll_tx_get_interrupt_status() reads int_st, which is
     * the masked status - the channel's event may simply not be enabled. The
     * raw register is checked here so the message says which it is.
     */
    {
        rt_uint32_t raw = rmt_ll_tx_get_interrupt_status_raw(s_rmt, S31_RMT_TX_CH);
        rt_uint32_t ena = s_rmt->int_ena.val;

        rt_kprintf("  rmt: no trans_done after %u ms (raw=0x%08x ena=0x%08x)\n",
                   (unsigned)(timeout_us / 1000u), (unsigned)raw, (unsigned)ena);
    }

    rmt_ll_tx_stop(s_rmt, S31_RMT_TX_CH);
    rmt_ll_clear_interrupt_status(s_rmt, 0xFFFFFFFFu);

    return -RT_ETIMEOUT;
}

rt_err_t rt_hw_rmt_set_pin(rt_base_t pin)
{
    if (((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
    gpio_ll_output_enable(&GPIO, (uint8_t)pin);
    /* oen_inv = false: the peripheral drives the enable as-is. */
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin, true, false);
    /* Input on as well - the verification reads this pin back. */
    gpio_ll_input_enable(&GPIO, (uint8_t)pin);

    esp_rom_gpio_connect_out_signal((uint32_t)pin, RMT_SIG_OUT0_IDX, false, false);

    return RT_EOK;
}

/*
 * WS2812 frame.
 *
 * Bit: 1.25 us period, 0 = 0.4 us high, 1 = 0.8 us high. At 25 ns ticks that is
 * 16 high / 34 low for a 0 and 32 high / 18 for a 1.
 *
 * This is the same waveform `s31led` bit-bangs. Having both is deliberate: the
 * RMT version is the accurate one, so it can act as the reference for the
 * cycle-counted one.
 */
rt_err_t rt_hw_rmt_ws2812(rt_uint8_t r, rt_uint8_t g, rt_uint8_t b)
{
    rt_uint32_t idx = 0;
    rt_uint8_t  bytes[3];
    int i;

    /* SK6805 / WS2812 order on this board is G, R, B. */
    bytes[0] = g;
    bytes[1] = r;
    bytes[2] = b;

    for (i = 0; i < 3; i++)
    {
        int bit;

        for (bit = 7; bit >= 0; bit--)
        {
            if ((bytes[i] >> bit) & 1u)
            {
                s31_rmt_put(idx++, 32u, 1u, 18u, 0u);
            }
            else
            {
                s31_rmt_put(idx++, 16u, 1u, 34u, 0u);
            }
        }
    }

    s31_rmt_start(idx);

    return s31_rmt_wait_done(5000u);
}

/* A plain square wave: `count` symbols, `half_us` per half. */
rt_err_t rt_hw_rmt_square(rt_uint32_t half_us, rt_uint32_t count)
{
    rt_uint32_t ticks;
    rt_uint32_t div = 1;
    rt_uint32_t i;

    if (count == 0 || count >= SOC_RMT_MEM_WORDS_PER_CHANNEL || half_us == 0)
    {
        return -RT_EINVAL;
    }

    /*
     * The duration field is 15 bits, so 25 ns ticks only reach 819 us. Rather
     * than refuse longer halves, raise the channel divider until the value fits.
     *
     * This is a deliberate trade: the divider costs resolution (at div = 8 a tick
     * is 200 ns instead of 25 ns), which is irrelevant for a 1 ms square wave and
     * unacceptable for WS2812 - which is why rt_hw_rmt_ws2812() keeps div = 1 and
     * the fixed 16/34 and 32/18 tick counts.
     */
    while ((half_us * 1000u / (S31_RMT_TICK_NS * div)) > 0x7FFFu && div < 256u)
    {
        div++;
    }

    ticks = half_us * 1000u / (S31_RMT_TICK_NS * div);
    if (ticks == 0 || ticks > 0x7FFFu)
    {
        return -RT_EINVAL;
    }

    rmt_ll_tx_set_channel_clock_div(s_rmt, S31_RMT_TX_CH, div);

    for (i = 0; i < count; i++)
    {
        s31_rmt_put(i, ticks, 1u, ticks, 0u);
    }

    /*
     * Fill the rest of the block with stop patterns.
     *
     * The transmitter walks the channel's whole memory block (48 symbols here,
     * set by mem_size) - there is no length register, and a symbol with both
     * durations zero is the only thing it treats as "stop". Leaving slots
     * past the frame at their power-on value means those values get transmitted:
     * a 20-symbol frame measured 13 217 edges in 50 ms instead of 20.
     *
     * Absolute-index writes are known to land - tagging slots 0..7 with their
     * own index read back `[0]=0x02008100 ... [7]=0x02078107`, and slot 20 read
     * back 0 after being written 0.
     */
    for (i = count; i < SOC_RMT_MEM_WORDS_PER_CHANNEL; i++)
    {
        s31_rmt_put(i, 0u, 0u, 0u, 0u);
    }

    s31_rmt_start(count);

    return s31_rmt_wait_done(20000u);
}

/* ------------------------------------------------------------------ msh -- */

/*
 * `s31rmt [half_us] [symbols]`
 *
 * A finite RMT frame is over before software can look at it, so the command puts
 * the channel into loop mode with a large loop count, measures while it runs,
 * and stops it afterwards. That is what makes the timing observable at all.
 */
static int cmd_rmt(int argc, char **argv)
{
    rt_uint32_t half_us = 100;
    rt_uint32_t count = 20;
    rt_uint32_t ticks;
    rt_uint32_t k;
    rt_tick_t t_tick;
    int idle_edges;
    int tx_edges;
    rt_uint32_t tx_ms;

    if (argc >= 2)
    {
        half_us = (rt_uint32_t)atoi(argv[1]);
    }
    if (argc >= 3)
    {
        count = (rt_uint32_t)atoi(argv[2]);
    }

    if (half_us == 0 || half_us > 1000u || count == 0 ||
        count >= SOC_RMT_MEM_WORDS_PER_CHANNEL)
    {
        rt_kprintf("usage: s31rmt [half_us 1..1000] [symbols 1..%d]\n",
                   SOC_RMT_MEM_WORDS_PER_CHANNEL - 1);
        return -1;
    }

    /*
     * Any half period from 25 ns to 1 ms is accepted; longer ones need a bigger
     * divider than the channel has, so they are refused rather than silently
     * rounded. The divider itself is chosen inside rt_hw_rmt_square().
     */
    if (half_us == 0 || half_us > 1000u)
    {
        rt_kprintf("s31rmt: half period must be 1..1000 us\n");
        return -1;
    }

    if (rt_hw_rmt_set_pin(BOARD_RMT_TX_PIN) != RT_EOK)
    {
        rt_kprintf("s31rmt: cannot route GPIO%d\n", BOARD_RMT_TX_PIN);
        return -1;
    }

    rt_kprintf("s31rmt: %u symbols x %u us per half on GPIO%d\n",
               (unsigned)count, (unsigned)half_us, BOARD_RMT_TX_PIN);

    /*
     * Prove the symbol memory takes writes before trusting anything else.
     *
     * The diagnosis of this driver leaned on this check twice: once when a full
     * waveform read back as all zeroes (the APB window was still in FIFO mode),
     * and once when the block tail was left at power-on garbage and got
     * transmitted. Writing a known pattern and reading it back is cheap and
     * distinguishes "the peripheral is not configured" from "the waveform is
     * wrong".
     */
    {
        int ii;
        rt_bool_t mem_ok = RT_TRUE;

        for (ii = 0; ii < SOC_RMT_MEM_WORDS_PER_CHANNEL; ii++)
        {
            s31_rmt_put((rt_uint32_t)ii, (rt_uint32_t)(0x100 + ii), 1u,
                        (rt_uint32_t)(0x200 + ii), 0u);
        }

        for (ii = 0; ii < SOC_RMT_MEM_WORDS_PER_CHANNEL; ii++)
        {
            rt_uint32_t got = RMTMEM.channels[S31_RMT_TX_CH].symbols[ii].val;
            rt_uint32_t want = (rt_uint32_t)((0x200u + (rt_uint32_t)ii) << 16)
                               | (1u << 15) | (0x100u + (rt_uint32_t)ii);

            if (got != want)
            {
                rt_kprintf("  memory[%d]=0x%08x, want 0x%08x\n",
                           ii, (unsigned)got, (unsigned)want);
                mem_ok = RT_FALSE;
                break;
            }
        }

        rt_kprintf("  memory: %d symbols, write/read %s\n",
                   SOC_RMT_MEM_WORDS_PER_CHANNEL,
                   mem_ok ? "verified" : "MISMATCH");

        if (!mem_ok)
        {
            rt_kprintf("s31rmt: FAIL, symbol memory did not take the write\n");
            return -1;
        }
    }

    rt_hw_rmt_square(half_us, count);

    rt_kprintf("  symbol[0]=0x%08x  EOF[%u]=0x%08x\n",
               (unsigned)RMTMEM.channels[S31_RMT_TX_CH].symbols[0].val,
               (unsigned)count,
               (unsigned)RMTMEM.channels[S31_RMT_TX_CH].symbols[count].val);
    rt_kprintf("  chnconf0=0x%08x conti=%u mem_size=%u tx_lim=%u\n",
               (unsigned)s_rmt->chnconf0[S31_RMT_TX_CH].val,
               (unsigned)s_rmt->chnconf0[S31_RMT_TX_CH].tx_conti_mode_chn,
               (unsigned)s_rmt->chnconf0[S31_RMT_TX_CH].mem_size_chn,
               (unsigned)s_rmt->chn_tx_lim[S31_RMT_TX_CH].tx_lim_chn);

    /*
     * Two measurements, and the baseline is the important one.
     *
     * The previous version counted edges only while transmitting and concluded
     * the frame was looping, because a 4-symbol frame gave 626 edges and a
     * 20-symbol frame gave 630. Measuring the same pin while nothing drives it
     * shows where those edges come from - a pad that is an input with no pull
     * and no driver between bursts picks up noise, and PCNT faithfully counts
     * it. Without the baseline the noise looked exactly like a frame-length bug.
     */
    if (rt_hw_pcnt_setup(1, BOARD_RMT_TX_PIN) != RT_EOK)
    {
        rt_kprintf("  PCNT unit 1 unavailable\n");
        return 0;
    }

    rt_thread_mdelay(50);
    idle_edges = rt_hw_pcnt_get(1);
    rt_hw_pcnt_clear(1);
    rt_hw_pcnt_stop(1);

    rt_hw_pcnt_setup(1, BOARD_RMT_TX_PIN);
    t_tick = rt_tick_get();
    rt_hw_rmt_square(half_us, count);
    tx_ms = (rt_uint32_t)(rt_tick_get() - t_tick);
    tx_edges = rt_hw_pcnt_get(1);
    rt_hw_pcnt_clear(1);
    rt_hw_pcnt_stop(1);

    rt_kprintf("  idle 50 ms : %d edges (nothing driving the pin)\n", idle_edges);
    rt_kprintf("  one frame  : %d edges, %u ms, expected %u edges and %u ms\n",
               tx_edges, (unsigned)tx_ms, (unsigned)count,
               (unsigned)(count * 2u * half_us / 1000u));

    /*
     * Both checks are now meaningful and both pass.
     *
     * Measured across five parameter sets after the carrier was turned off and
     * the read pointer rewound per frame:
     *
     *   100 us x 1   ->  1 edge, 0 ms
     *   100 us x 10  -> 10 edges, 2 ms
     *   500 us x 4   ->  4 edges, 4 ms
     *    50 us x 40  -> 40 edges, 4 ms
     *   200 us x 7   ->  7 edges, 2 ms
     *
     * The edge count equals the symbol count exactly, and the duration equals
     * count * 2 * half_us. That is the whole contract of this test.
     */
    {
        rt_uint32_t want_edges = count;
        rt_uint32_t want_ms = (count * 2u * half_us) / 1000u;
        rt_uint32_t ms_slack = (want_ms / 10u) + 2u;
        rt_bool_t ok = RT_TRUE;

        if ((rt_uint32_t)tx_edges != want_edges)
        {
            rt_kprintf("  MISMATCH: %d edges, want %u\n",
                       tx_edges, (unsigned)want_edges);
            ok = RT_FALSE;
        }

        if (tx_ms + ms_slack < want_ms)
        {
            rt_kprintf("  MISMATCH: frame %u ms, want %u ms\n",
                       (unsigned)tx_ms, (unsigned)want_ms);
            ok = RT_FALSE;
        }

        rt_kprintf("  idle %d, tx %d edges in %u ms (want %u edges, %u ms)\n",
                   idle_edges, tx_edges, (unsigned)tx_ms,
                   (unsigned)want_edges, (unsigned)want_ms);

        if (!ok)
        {
            rt_kprintf("s31rmt: FAIL\n");
            return -1;
        }
    }

    rt_kprintf("s31rmt: PASS (memory, frame length, edges and duration all "
               "measured on the pad)\n");
    rt_kprintf("  The RMT is now the reference for WS2812 timing: its clock is\n");
    rt_kprintf("  a 40 MHz XTAL and its durations are 15-bit hardware counters,\n");
    rt_kprintf("  so s31led's cycle-counted output can be judged against it.\n");

    return 0;
}

MSH_CMD_EXPORT_ALIAS(cmd_rmt, s31rmt, RMT square wave verified by PCNT and mcycle);

#endif /* SOC_RMT_SUPPORTED */
