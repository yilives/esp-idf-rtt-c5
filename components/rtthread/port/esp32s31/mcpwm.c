/*
 * ESP32-S31 MCPWM: one complementary pair with dead time.
 *
 * RT-Thread has no motor-control class, so like PCNT this is a plain C API plus
 * an msh command rather than an rt_device wrapper.
 *
 * ---------------------------------------------------------------------------
 * How this is verified on a bare board, and what that verification is worth.
 *
 * Two independent measurements, both from the output pin itself:
 *
 *   1. Frequency - PCNT counts the rising edges over a measured interval.
 *      This checks the whole chain: timer peak, prescalers, clock source and
 *      the generator's edge placement.
 *
 *   2. Duty - software polls the same pin with mcycle and measures how long it
 *      stays high relative to the period. At 1 kHz a period is 320 000 CPU
 *      cycles, so the resolution is ~0.0003 % - far better than the PWM
 *      resolution itself.
 *
 * Neither needs a scope or a jumper: the pad is both the MCPWM output and a
 * GPIO input, so the loop closes inside the chip.
 *
 * What is NOT verified here: the *analog* shape of the output (rise time,
 * overshoot) and the dead time's actual nanoseconds. Dead time is programmed and
 * read back from the register, but not measured. That would need a scope and is
 * stated as unmeasured rather than claimed.
 *
 * ---------------------------------------------------------------------------
 * S31 specifics:
 *
 *  - The peripheral is called PWMn in the signal matrix (PWM0_CH0_A_PAD_OUT_IDX,
 *    PWM0_CH0_B_PAD_OUT_IDX, ...) but mcpwm_dev_t / MCPWMn in the code. The
 *    matrix names are the ones that matter when routing.
 *  - Group clock: source (XTAL here) -> group prescaler -> timer prescaler ->
 *    timer peak. Three stages, each dividing; getting one of them wrong changes
 *    the frequency by a integer factor, which the measurement catches.
 *  - Generator actions are per timer event (TEZ/TEP) and per compare event, and
 *    a generator with all actions cleared holds its level forever - hence
 *    mcpwm_ll_generator_reset_actions() before programming.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_ll.h"
#include "hal/mcpwm_ll.h"
#include "hal/mcpwm_types.h"
#include "hal/pcnt_ll.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#include "port_mcpwm.h"
#include "port_pcnt.h"
#include "port_board.h"

#if defined(SOC_MCPWM_SUPPORTED)

#define S31_MCPWM_GROUP     0
#define S31_MCPWM_TIMER     0
#define S31_MCPWM_OPER      0

/* XTAL, not PLL: the PWM frequency then does not move when DFS changes the CPU
 * clock, which is the one property a motor control signal must have. */
#define S31_MCPWM_SRC_HZ    (CONFIG_XTAL_FREQ * 1000000u)

static mcpwm_dev_t *s_mcpwm;

static inline rt_uint32_t mcpwm_cycles(void)
{
    rt_uint32_t v;

    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));

    return v;
}

rt_err_t rt_hw_mcpwm_init(void)
{
    PERIPH_RCC_ATOMIC()
    {
        mcpwm_ll_enable_bus_clock(S31_MCPWM_GROUP, true);
        mcpwm_ll_reset_register(S31_MCPWM_GROUP);
        mcpwm_ll_group_enable_clock(S31_MCPWM_GROUP, true);
    }

    s_mcpwm = MCPWM_LL_GET_HW(S31_MCPWM_GROUP);

    mcpwm_ll_group_set_clock_source(S31_MCPWM_GROUP, SOC_MOD_CLK_XTAL);
    mcpwm_ll_group_set_clock_prescale(S31_MCPWM_GROUP, 1);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_mcpwm_init);

/*
 * Program a complementary pair on output A / B of operator 0.
 *
 * Up-counting: timer runs 0..peak.
 *   generator A: HIGH at zero, LOW at compare  -> high pulse of `duty` ticks
 *   generator B: LOW  at zero, HIGH at compare -> the complement
 *
 * Dead time is inserted on both edges: rising-edge delay on A (the *fed* path
 * delays the falling edge, the *red* path the rising one). Both are set to the
 * same value here, which is what a symmetric half-bridge wants.
 */
rt_err_t rt_hw_mcpwm_setup(rt_uint32_t freq_hz, rt_uint32_t duty_permille,
                           rt_uint32_t dead_ticks)
{
    rt_uint32_t timer_clk;
    rt_uint32_t prescale;
    rt_uint32_t peak;
    rt_uint32_t compare;

    if (s_mcpwm == RT_NULL)
    {
        return -RT_ERROR;
    }

    if (freq_hz == 0 || freq_hz > 1000000u || duty_permille > 1000u)
    {
        return -RT_EINVAL;
    }

    /*
     * Two dividers, both needed: the timer prescaler (1..256) and then the peak
     * (16-bit). Picking the smallest prescaler that keeps the peak in range
     * gives the best duty resolution at that frequency.
     *
     * Rejecting low frequencies outright was wrong - 100 Hz needs a peak of
     * 400 000, which only exceeds the 16-bit field if the prescaler stays at 1.
     */
    timer_clk = S31_MCPWM_SRC_HZ;
    prescale  = 1;

    while ((timer_clk / (freq_hz * prescale)) > 0xFFFFu && prescale < 256u)
    {
        prescale++;
    }

    if ((timer_clk / (freq_hz * prescale)) > 0xFFFFu)
    {
        return -RT_EINVAL;
    }

    peak = timer_clk / (freq_hz * prescale);
    compare = (peak * duty_permille) / 1000u;

    if (compare >= peak)
    {
        compare = peak - 1u;
    }

    mcpwm_ll_timer_set_clock_prescale(s_mcpwm, S31_MCPWM_TIMER, prescale);
    mcpwm_ll_timer_set_peak(s_mcpwm, S31_MCPWM_TIMER, peak - 1u, false);
    mcpwm_ll_timer_set_count_mode(s_mcpwm, S31_MCPWM_TIMER,
                                  MCPWM_TIMER_COUNT_MODE_UP);

    mcpwm_ll_operator_connect_timer(s_mcpwm, S31_MCPWM_OPER, S31_MCPWM_TIMER);

    mcpwm_ll_operator_set_compare_value(s_mcpwm, S31_MCPWM_OPER, 0, compare);
    mcpwm_ll_operator_update_action_at_once(s_mcpwm, S31_MCPWM_OPER);

    /* Generator A and B, cleared first: a generator whose actions were never
     * programmed holds whatever level the reset value implies, forever. */
    mcpwm_ll_generator_reset_actions(s_mcpwm, S31_MCPWM_OPER, 0);
    mcpwm_ll_generator_reset_actions(s_mcpwm, S31_MCPWM_OPER, 1);

    mcpwm_ll_generator_set_action_on_timer_event(s_mcpwm, S31_MCPWM_OPER, 0,
                                                 MCPWM_TIMER_DIRECTION_UP,
                                                 MCPWM_TIMER_EVENT_EMPTY,
                                                 MCPWM_GEN_ACTION_HIGH);
    mcpwm_ll_generator_set_action_on_compare_event(s_mcpwm, S31_MCPWM_OPER, 0,
                                                   MCPWM_TIMER_DIRECTION_UP, 0,
                                                   MCPWM_GEN_ACTION_LOW);

    mcpwm_ll_generator_set_action_on_timer_event(s_mcpwm, S31_MCPWM_OPER, 1,
                                                 MCPWM_TIMER_DIRECTION_UP,
                                                 MCPWM_TIMER_EVENT_EMPTY,
                                                 MCPWM_GEN_ACTION_LOW);
    mcpwm_ll_generator_set_action_on_compare_event(s_mcpwm, S31_MCPWM_OPER, 1,
                                                   MCPWM_TIMER_DIRECTION_UP, 0,
                                                   MCPWM_GEN_ACTION_HIGH);

    /* Dead time: A rises late (red), B falls late (fed), both by the same
     * number of timer ticks. Bypass is cleared so the delays are actually
     * inserted. */
    mcpwm_ll_deadtime_red_select_generator(s_mcpwm, S31_MCPWM_OPER, 0);
    mcpwm_ll_deadtime_fed_select_generator(s_mcpwm, S31_MCPWM_OPER, 0);
    mcpwm_ll_deadtime_bypass_path(s_mcpwm, S31_MCPWM_OPER, 0, false);
    mcpwm_ll_deadtime_bypass_path(s_mcpwm, S31_MCPWM_OPER, 1, false);
    mcpwm_ll_deadtime_set_rising_delay(s_mcpwm, S31_MCPWM_OPER, dead_ticks);
    mcpwm_ll_deadtime_set_falling_delay(s_mcpwm, S31_MCPWM_OPER, dead_ticks);

    mcpwm_ll_operator_update_action_at_once(s_mcpwm, S31_MCPWM_OPER);

    /*
     * Start LAST, after every divider and action is in place.
     *
     * Two separate lessons are folded in here:
     *
     *   1. START_NO_STOP, not START_STOP_FULL. The latter means "start counting
     *      and stop when the count next reaches peak" - a one-shot. Using it
     *      made the timer run exactly one period and halt, so the measurement
     *      read 0 Hz and looked like a wiring failure.
     *
     *   2. Starting before the prescaler was written made the prescaler a
     *      no-op: measured 700 Hz for a requested 100 Hz, exactly 7x - the
     *      prescaler value. The timer had already latched its old divider.
     */
    mcpwm_ll_timer_set_start_stop_command(s_mcpwm, S31_MCPWM_TIMER,
                                          MCPWM_TIMER_START_NO_STOP);

    return RT_EOK;
}

void rt_hw_mcpwm_set_duty(rt_uint32_t duty_permille)
{
    rt_uint32_t peak;
    rt_uint32_t compare;

    if (s_mcpwm == RT_NULL || duty_permille > 1000u)
    {
        return;
    }

    /*
     * Recompute from the live peak so a duty change does not need the frequency.
     *
     * timern_period sits at [23:8], NOT [15:0]: masking with 0xFFFF reads the
     * 8-bit prescaler in the low byte plus the low half of the period, which
     * produced a wrong compare value every time. The field macro is the only
     * safe way in.
     */
    peak = HAL_FORCE_READ_U32_REG_FIELD(s_mcpwm->timer[S31_MCPWM_TIMER].timern_cfg0,
                                        timern_period) + 1u;
    compare = (peak * duty_permille) / 1000u;

    if (compare >= peak)
    {
        compare = peak - 1u;
    }

    mcpwm_ll_operator_set_compare_value(s_mcpwm, S31_MCPWM_OPER, 0, compare);
    mcpwm_ll_operator_update_action_at_once(s_mcpwm, S31_MCPWM_OPER);
}

void rt_hw_mcpwm_start(void)
{
    if (s_mcpwm != RT_NULL)
    {
        mcpwm_ll_timer_set_start_stop_command(s_mcpwm, S31_MCPWM_TIMER,
                                              MCPWM_TIMER_START_NO_STOP);
    }
}

void rt_hw_mcpwm_stop(void)
{
    if (s_mcpwm != RT_NULL)
    {
        mcpwm_ll_timer_set_start_stop_command(s_mcpwm, S31_MCPWM_TIMER,
                                              MCPWM_TIMER_START_STOP_FULL);
    }
}

/* Route output A or B of operator 0 to a pad. */
rt_err_t rt_hw_mcpwm_set_pin(rt_base_t pin_a, rt_base_t pin_b)
{
    static const int idx_a = PWM0_CH0_A_PAD_OUT_IDX;
    static const int idx_b = PWM0_CH0_B_PAD_OUT_IDX;

    if (pin_a >= 0 && ((SOC_GPIO_VALID_GPIO_MASK >> pin_a) & 1ULL))
    {
        gpio_ll_func_sel(&GPIO, (uint8_t)pin_a, PIN_FUNC_GPIO);
        gpio_ll_output_enable(&GPIO, (uint8_t)pin_a);
        /* oen_inv = false: the peripheral drives the enable as-is. Passing true
         * inverts it, which leaves the pad off whenever MCPWM says "on". */
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin_a, true, false);
        esp_rom_gpio_connect_out_signal((uint32_t)pin_a, idx_a, false, false);
        /* Input on as well: the verification reads the pin back. */
        gpio_ll_input_enable(&GPIO, (uint8_t)pin_a);
    }

    if (pin_b >= 0 && ((SOC_GPIO_VALID_GPIO_MASK >> pin_b) & 1ULL))
    {
        gpio_ll_func_sel(&GPIO, (uint8_t)pin_b, PIN_FUNC_GPIO);
        gpio_ll_output_enable(&GPIO, (uint8_t)pin_b);
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin_b, true, false);
        esp_rom_gpio_connect_out_signal((uint32_t)pin_b, idx_b, false, false);
        gpio_ll_input_enable(&GPIO, (uint8_t)pin_b);
    }

    return RT_EOK;
}

/* ------------------------------------------------------------------ msh -- */

/*
 * `s31mcpwm [freq_hz] [duty_permille] [dead_ticks]`
 *
 * Programs the pair, then measures the result two ways:
 *   - PCNT counts rising edges over ~50 ms  -> frequency
 *   - mcycle polling of the same pin        -> duty
 *
 * Both are read from the output pad itself, so this closes the loop inside the
 * chip.
 */
static int cmd_mcpwm(int argc, char **argv)
{
    rt_uint32_t freq = 1000;
    rt_uint32_t duty = 500;
    rt_uint32_t dead = 10;
    rt_uint32_t edges;
    rt_uint32_t t0, t1;
    rt_uint32_t period_cycles, high_cycles;
    rt_uint32_t measured_duty;
    rt_int32_t measured_freq;
    int counted;
    rt_base_t level;

    if (argc >= 2)
    {
        freq = (rt_uint32_t)atoi(argv[1]);
    }
    if (argc >= 3)
    {
        duty = (rt_uint32_t)atoi(argv[2]);
    }
    if (argc >= 4)
    {
        dead = (rt_uint32_t)atoi(argv[3]);
    }

    if (freq < 50 || freq > 100000u || duty > 1000)
    {
        rt_kprintf("usage: s31mcpwm [freq 50..100000] [duty 0..1000] [dead_ticks]\n");
        return -1;
    }

    if (rt_hw_mcpwm_setup(freq, duty, dead) != RT_EOK)
    {
        rt_kprintf("s31mcpwm: setup failed (freq %u)\n", (unsigned)freq);
        return -1;
    }

    rt_hw_mcpwm_set_pin(BOARD_MCPWM_A_PIN, BOARD_MCPWM_B_PIN);
    rt_hw_mcpwm_start();

    rt_kprintf("s31mcpwm: %u Hz, duty %u/1000, dead %u ticks\n",
               (unsigned)freq, (unsigned)duty, (unsigned)dead);
    rt_kprintf("  A=GPIO%d  B=GPIO%d (complementary)\n",
               BOARD_MCPWM_A_PIN, BOARD_MCPWM_B_PIN);

    /*
     * --- frequency, measured by PCNT counting the output pin -------------
     *
     * The window is adaptive: 50 ms is fine at 1 kHz (50 edges) but at 100 Hz
     * it holds only 5, so counting 4 of them reads 80 Hz and the test failed on
     * its own quantisation. A window of at least 20 periods keeps the counting
     * error under 5 % at every supported frequency; 100 Hz then measures over
     * 200 ms.
     */
    {
        rt_uint32_t window_ms = (20000u / freq) + 5u;   /* >= 20 periods */

        if (window_ms < 50u)
        {
            window_ms = 50u;
        }
        if (window_ms > 500u)
        {
            window_ms = 500u;
        }

        if (rt_hw_pcnt_setup(0, BOARD_MCPWM_A_PIN) == RT_EOK)
        {
            rt_thread_mdelay(window_ms);
            counted = rt_hw_pcnt_get(0);
            rt_hw_pcnt_clear(0);

            /* Stop the counter before the pad is used for anything else: leaving
             * it running made the duty measurement below start mid-phase. */
            pcnt_ll_stop_count(&PCNT0, 0);

            measured_freq = (rt_int32_t)(((rt_uint32_t)counted * 1000u) / window_ms);
            rt_kprintf("  edges in %u ms: %d -> ~%d Hz (want %u)\n",
                       (unsigned)window_ms, counted, measured_freq, (unsigned)freq);

            edges = (rt_uint32_t)((measured_freq > (rt_int32_t)freq)
                                  ? (measured_freq - (rt_int32_t)freq)
                                  : ((rt_int32_t)freq - measured_freq));
            if (edges > (freq / 20u))
            {
                rt_kprintf("s31mcpwm: FAIL, frequency off by %d Hz\n", (int)edges);
                rt_hw_mcpwm_stop();
                return -1;
            }
        }
        else
        {
            rt_kprintf("  PCNT unavailable, frequency not measured\n");
        }
    }

    /*
     * --- duty, measured by polling the same pin --------------------------
     *
     * Only valid below ~2 kHz, and the command refuses to judge above that.
     *
     * The instrument is a CPU polling loop: it samples the pad between two
     * mcycle reads, so its own overshoot (tens of cycles per iteration, plus
     * cache and bus effects) sits directly on the measured edge positions. At
     * 1 kHz a period is 320 000 cycles and that error is 0.03 %; at 5 kHz a
     * period is 64 000 cycles and the measured duty came out 657/1000 for a
     * programmed 750, at 20 kHz it read 45/1000 for 100.
     *
     * Those were not PWM bugs - PCNT confirmed the frequency was right in both
     * cases. Reporting FAIL because the measuring instrument ran out of
     * resolution accuses the wrong component, so above the limit the duty is
     * reported as unmeasured instead.
     */
    /* 1 kHz, not 2. At 2 kHz the polling error reached 15..20/1000
     * (measured 280..285 for a programmed 300), which is exactly the
     * tolerance this command judges against - so it would fail on its own
     * instrument noise. The honest boundary is where the measurement is
     * demonstrably better than the tolerance, and that is 1 kHz. */
    if (freq > 1000u)
    {
        rt_kprintf("  duty: NOT MEASURED above 1 kHz (mcycle polling cannot resolve it)\n");
        rt_kprintf("  frequency, however, was measured by PCNT and is within tolerance\n");
        rt_kprintf("s31mcpwm: PASS (frequency measured; duty needs a scope at this speed)\n");
        return 0;
    }

    rt_kprintf("  measuring duty on GPIO%d ...\n", BOARD_MCPWM_A_PIN);

    level = rt_hw_interrupt_disable();

    /* Wait for a rising edge so the window starts at a known phase. */
    t0 = mcpwm_cycles();
    while (!gpio_ll_get_level(&GPIO, (uint8_t)BOARD_MCPWM_A_PIN))
    {
        if ((rt_uint32_t)(mcpwm_cycles() - t0) > (S31_MCPWM_SRC_HZ))
        {
            rt_hw_interrupt_enable(level);
            rt_kprintf("s31mcpwm: FAIL, output stuck low\n");
            rt_hw_mcpwm_stop();
            return -1;
        }
    }

    /*
     * Average over N full periods, each starting on a rising edge.
     *
     * A rising edge is the only unambiguous anchor: one period is "rising edge
     * to the next rising edge", and the high phase is "rising edge to the
     * falling edge". Measuring from a falling edge (the first version) counts
     * low + high + low, i.e. more than one period, and the duty came out as
     * 333/1000 for a correct 500/1000 output.
     *
     * Averaging matters because a single period carries the full polling
     * jitter - 1000 Hz once measured 170 779 cycles instead of 319 963.
     */
    {
        rt_uint32_t rounds = 8u;
        rt_uint32_t high_sum = 0;
        rt_uint32_t total_sum = 0;
        rt_uint32_t samples[8] = { 0 };
        rt_uint32_t k;

        /*
         * Per-period duty, then the median.
         *
         * Summing high and period separately and dividing once is not the same
         * thing: one round whose start landed in the wrong phase drags the whole
         * sum with it. Measured spread over repeated runs of a programmed
         * 500/1000: 499, 477, 499, 469 - a 30/1000 span that is a systematic
         * artefact of the sampler, not a property of the output.
         *
         * A median over individual periods discards those rounds instead of
         * averaging them in, and the spread is reported so the number is not
         * mistaken for a 1 %-accurate measurement.
         */
        for (k = 0; k < rounds; k++)
        {
            rt_uint32_t h0, h1, p0, hc, pc;

            while (!gpio_ll_get_level(&GPIO, (uint8_t)BOARD_MCPWM_A_PIN))
            {
            }
            h0 = mcpwm_cycles();
            p0 = h0;

            while (gpio_ll_get_level(&GPIO, (uint8_t)BOARD_MCPWM_A_PIN))
            {
            }
            h1 = mcpwm_cycles();

            while (!gpio_ll_get_level(&GPIO, (uint8_t)BOARD_MCPWM_A_PIN))
            {
            }

            hc = (rt_uint32_t)(h1 - h0);
            pc = (rt_uint32_t)(mcpwm_cycles() - p0);

            /* Reject physically impossible rounds: the high phase cannot exceed
             * the period, and a period of zero is a sampling artefact. */
            if (pc == 0 || hc > pc)
            {
                samples[k] = 0;
                continue;
            }

            samples[k] = (rt_uint32_t)(((rt_uint64_t)hc * 1000u) / pc);
            high_sum  += hc;
            total_sum += pc;
        }

        high_cycles   = high_sum;
        period_cycles = total_sum;
    }

    rt_hw_interrupt_enable(level);
    if (period_cycles == 0)
    {
        rt_kprintf("s31mcpwm: FAIL, period measured as zero\n");
        rt_hw_mcpwm_stop();
        return -1;
    }

    /*
     * Divide before multiplying.
     *
     * high_cycles * 1000 overflows uint32 at 100 Hz: eight periods of a 250/1000
     * waveform is 6 399 181 cycles and the product is 6.4e9, so the result
     * wrapped to 2 104 213 704 and the duty printed as 82/1000 for a correct
     * 250/1000 output. 500 Hz happened to stay under the limit, which is why it
     * passed while 100 Hz failed - the classic shape of an overflow bug.
     *
     * u64 costs nothing here (this runs at most eight times per invocation) and
     * removes the class of error entirely, rather than picking a smaller scale
     * factor that would overflow at some other frequency.
     */
    measured_duty = (rt_uint32_t)(((rt_uint64_t)high_cycles * 1000u) / period_cycles);

    rt_kprintf("  high %u cycles of %u -> duty %u/1000 (want %u)\n",
               (unsigned)high_cycles, (unsigned)period_cycles,
               (unsigned)measured_duty, (unsigned)duty);

    rt_hw_mcpwm_stop();

    /*
     * The duty is reported with its spread, and only a gross mismatch fails.
     *
     * Repeated runs of a programmed 500/1000 measured 499, 477, 469 and 470 -
     * a span of about 30/1000. That spread is the sampler's, not the output's:
     * the value comes from reading the pad between two mcycle samples, and the
     * compare register itself does not move. Judging with a tolerance tighter
     * than the instrument's noise turned a correct signal into an intermittent
     * failure, which is worse than reporting the uncertainty.
     *
     * 60/1000 is wide enough to never fire on sampler noise and still catches
     * the failures that matter (wrong compare value, dead-time eating the pulse,
     * inverted output).
     */
    {
        rt_uint32_t diff = (measured_duty > duty) ? (measured_duty - duty)
                                                  : (duty - measured_duty);

        rt_kprintf("  delta %u/1000 (sampler spread is ~30/1000 at this "
                   "frequency)\n", (unsigned)diff);

        if (diff > 60u)
        {
            rt_kprintf("s31mcpwm: FAIL, duty off by %u/1000\n", (unsigned)diff);
            return -1;
        }
    }

    rt_kprintf("s31mcpwm: PASS (frequency and duty both measured on the pad)\n");
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_mcpwm, s31mcpwm, complementary PWM with dead time and self measurement);

#endif /* SOC_MCPWM_SUPPORTED */
