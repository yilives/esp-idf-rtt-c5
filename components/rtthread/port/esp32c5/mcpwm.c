/*
 * ESP32-C5 MCPWM: one complementary pair with dead time.
 *
 * RT-Thread has no motor-control class, so like RMT this is a plain C API plus an
 * msh command rather than an rt_device wrapper.
 *
 * ---------------------------------------------------------------------------
 * How this is verified on a bare board, and what that verification is worth.
 *
 * Two independent measurements, both taken from the output pin itself:
 *
 *   1. Frequency - the PCNT unit from port/esp32c5/pcnt.c counts the rising
 *      edges over a measured interval. That checks the whole chain: timer peak,
 *      both prescalers, clock source and the generator's edge placement.
 *
 *   2. Duty - software polls the same pin with mcycle and measures how long it
 *      stays high relative to the period. At 1 kHz a period is 240 000 CPU
 *      cycles, so the resolution is far better than the PWM resolution itself.
 *
 * Neither needs a scope or a jumper: the pad is both the MCPWM output and a GPIO
 * input, so the loop closes inside the chip. Better still, the counter used for
 * measurement is the one from 搂17d, which was verified independently - so the two
 * drivers check each other rather than one timestamping itself.
 *
 * What is NOT verified: the analog shape of the output (rise time, overshoot) and
 * the dead time's actual nanoseconds. Dead time is programmed and read back from
 * the register, but not measured - that needs a scope, so it is stated as
 * unmeasured rather than claimed.
 *
 * ---------------------------------------------------------------------------
 * C5 specifics, versus the S31 file this was ported from:
 *
 *  - The signal matrix calls the outputs PWMn, not MCPWMn, and the C5 names are
 *    shorter: PWM0_OUT0A_IDX / PWM0_OUT0B_IDX, not the S31's
 *    PWM0_CH0_A_PAD_OUT_IDX / PWM0_CH0_B_PAD_OUT_IDX.
 *
 *  - One group (MCPWM0); MCPWM_LL_GET_HW() returns NULL for anything else.
 *    3 timers, 3 operators, 2 generators per operator.
 *
 *  - `mcpwm_ll_group_set_clock_source()` takes a `soc_module_clk_t`
 *    (SOC_MOD_CLK_XTAL), not a `mcpwm_clock_source_t` - the S31 signature differs.
 *
 *  - XTAL, not the F160M PLL: the PWM frequency then does not move when DFS
 *    changes the CPU clock, which is the one property a motor-control signal must
 *    have. It is also the source that is actually running in this image - four
 *    drivers in this port have now been bitten by a default that points at an
 *    unpowered PLL.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/mcpwm_struct.h"
#include "hal/gpio_ll.h"
#include "hal/mcpwm_ll.h"
#include "hal/mcpwm_types.h"
#include "esp_rom_gpio.h"
#include "esp_private/esp_clk.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_MCPWM_SUPPORTED)

#include "port_mcpwm.h"
#include "port_pcnt.h"

#define C5_MCPWM_GROUP      0
#define C5_MCPWM_TIMER      0
#define C5_MCPWM_OPER       0

static mcpwm_dev_t *s_mcpwm;

static inline rt_uint32_t c5_mcpwm_cycles(void)
{
    rt_uint32_t v;

    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));

    return v;
}

rt_err_t rt_hw_mcpwm_init(void)
{
    PERIPH_RCC_ATOMIC()
    {
        mcpwm_ll_enable_bus_clock(C5_MCPWM_GROUP, true);
        mcpwm_ll_reset_register(C5_MCPWM_GROUP);
        mcpwm_ll_group_enable_clock(C5_MCPWM_GROUP, true);
    }

    s_mcpwm = MCPWM_LL_GET_HW(C5_MCPWM_GROUP);

    mcpwm_ll_group_set_clock_source(C5_MCPWM_GROUP, SOC_MOD_CLK_XTAL);
    mcpwm_ll_group_set_clock_prescale(C5_MCPWM_GROUP, 1);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_mcpwm_init);

/*
 * Program a complementary pair on output A / B of operator 0.
 *
 * Up-counting: the timer runs 0..peak.
 *   generator A: HIGH at zero, LOW at compare  -> high pulse of `duty` ticks
 *   generator B: LOW  at zero, HIGH at compare -> the complement
 *
 * Dead time is inserted on both edges: the rising-edge delay on A (the *red*
 * path delays the rising edge, the *fed* path the falling one). Both are set to
 * the same value here, which is what a symmetric half-bridge wants.
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
     * (16-bit). Picking the smallest prescaler that keeps the peak in range gives
     * the best duty resolution at that frequency.
     *
     * Rejecting low frequencies outright would be wrong - 100 Hz needs a peak of
     * 480 000, which only exceeds the 16-bit field if the prescaler stays at 1.
     */
    timer_clk = (rt_uint32_t)esp_clk_xtal_freq();
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
    if (peak == 0u)
    {
        return -RT_EINVAL;
    }

    compare = (peak * duty_permille) / 1000u;
    if (compare >= peak)
    {
        compare = peak - 1u;
    }

    mcpwm_ll_timer_set_clock_prescale(s_mcpwm, C5_MCPWM_TIMER, (int)prescale);
    mcpwm_ll_timer_set_peak(s_mcpwm, C5_MCPWM_TIMER, peak - 1u, false);
    mcpwm_ll_timer_set_count_mode(s_mcpwm, C5_MCPWM_TIMER,
                                  MCPWM_TIMER_COUNT_MODE_UP);

    mcpwm_ll_operator_connect_timer(s_mcpwm, C5_MCPWM_OPER, C5_MCPWM_TIMER);

    mcpwm_ll_operator_set_compare_value(s_mcpwm, C5_MCPWM_OPER, 0, compare);
    mcpwm_ll_operator_update_action_at_once(s_mcpwm, C5_MCPWM_OPER);

    /* Generators A and B, cleared first: a generator whose actions were never
     * programmed holds whatever level the reset value implies, forever. */
    mcpwm_ll_generator_reset_actions(s_mcpwm, C5_MCPWM_OPER, 0);
    mcpwm_ll_generator_reset_actions(s_mcpwm, C5_MCPWM_OPER, 1);

    mcpwm_ll_generator_set_action_on_timer_event(s_mcpwm, C5_MCPWM_OPER, 0,
                                                 MCPWM_TIMER_DIRECTION_UP,
                                                 MCPWM_TIMER_EVENT_EMPTY,
                                                 MCPWM_GEN_ACTION_HIGH);
    mcpwm_ll_generator_set_action_on_compare_event(s_mcpwm, C5_MCPWM_OPER, 0,
                                                   MCPWM_TIMER_DIRECTION_UP, 0,
                                                   MCPWM_GEN_ACTION_LOW);

    mcpwm_ll_generator_set_action_on_timer_event(s_mcpwm, C5_MCPWM_OPER, 1,
                                                 MCPWM_TIMER_DIRECTION_UP,
                                                 MCPWM_TIMER_EVENT_EMPTY,
                                                 MCPWM_GEN_ACTION_LOW);
    mcpwm_ll_generator_set_action_on_compare_event(s_mcpwm, C5_MCPWM_OPER, 1,
                                                   MCPWM_TIMER_DIRECTION_UP, 0,
                                                   MCPWM_GEN_ACTION_HIGH);

    /* Dead time: A rises late (red), B falls late (fed), both by the same number
     * of timer ticks. Bypass is cleared so the delays are actually inserted. */
    mcpwm_ll_deadtime_red_select_generator(s_mcpwm, C5_MCPWM_OPER, 0);
    mcpwm_ll_deadtime_fed_select_generator(s_mcpwm, C5_MCPWM_OPER, 0);
    mcpwm_ll_deadtime_bypass_path(s_mcpwm, C5_MCPWM_OPER, 0, false);
    mcpwm_ll_deadtime_bypass_path(s_mcpwm, C5_MCPWM_OPER, 1, false);
    mcpwm_ll_deadtime_set_rising_delay(s_mcpwm, C5_MCPWM_OPER, dead_ticks);
    mcpwm_ll_deadtime_set_falling_delay(s_mcpwm, C5_MCPWM_OPER, dead_ticks);

    mcpwm_ll_operator_update_action_at_once(s_mcpwm, C5_MCPWM_OPER);

    /*
     * Start LAST, after every divider and action is in place.
     *
     * START_NO_STOP, not START_STOP_FULL. The latter means "start counting and
     * stop when the count next reaches peak" - a one-shot. Using it makes the
     * timer run exactly one period and halt, so the measurement reads 0 Hz and
     * looks like a wiring failure. Starting before the prescaler is written makes
     * the prescaler a no-op, because the timer has already latched its old
     * divider. Both were measured on the S31 file this derives from.
     */
    mcpwm_ll_timer_set_start_stop_command(s_mcpwm, C5_MCPWM_TIMER,
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

    /* Recompute from the live peak so a duty change does not need the frequency.
     *
     * timern_period sits at [23:8], NOT [15:0]: masking with 0xFFFF reads the
     * 8-bit prescaler in the low byte plus the low half of the period, which
     * produces a wrong compare value every time. The field macro is the only safe
     * way in. */
    /* The field name differs between chips: the C5 register struct calls it
     * `timer_cfg0`, while the S31 file this was ported from uses `timern_cfg0`.
     * The field inside (`timern_period`) keeps the "n" because it is named per
     * timer in the register description. Two near-identical names one letter
     * apart is exactly the kind of thing the compiler catches only if you look. */
    peak = HAL_FORCE_READ_U32_REG_FIELD(s_mcpwm->timer[C5_MCPWM_TIMER].timer_cfg0,
                                        timern_period) + 1u;
    compare = (peak * duty_permille) / 1000u;

    if (compare >= peak)
    {
        compare = peak - 1u;
    }

    mcpwm_ll_operator_set_compare_value(s_mcpwm, C5_MCPWM_OPER, 0, compare);
    mcpwm_ll_operator_update_action_at_once(s_mcpwm, C5_MCPWM_OPER);
}

void rt_hw_mcpwm_start(void)
{
    if (s_mcpwm != RT_NULL)
    {
        mcpwm_ll_timer_set_start_stop_command(s_mcpwm, C5_MCPWM_TIMER,
                                              MCPWM_TIMER_START_NO_STOP);
    }
}

void rt_hw_mcpwm_stop(void)
{
    if (s_mcpwm != RT_NULL)
    {
        mcpwm_ll_timer_set_start_stop_command(s_mcpwm, C5_MCPWM_TIMER,
                                              MCPWM_TIMER_START_STOP_FULL);
    }
}

/* Route output A or B of operator 0 to a pad. */
rt_err_t rt_hw_mcpwm_set_pin(rt_base_t pin_a, rt_base_t pin_b)
{
    if (pin_a >= 0 && pin_a < SOC_GPIO_PIN_COUNT &&
        ((SOC_GPIO_VALID_GPIO_MASK >> pin_a) & 1ULL))
    {
        gpio_ll_func_sel(&GPIO, (uint8_t)pin_a, PIN_FUNC_GPIO);
        gpio_ll_output_enable(&GPIO, (uint8_t)pin_a);
        /* oen_inv = false: the peripheral drives the enable as-is. Passing true
         * inverts it, which leaves the pad off whenever MCPWM says "on" - the
         * same trap that cost the RMT driver a whole round (搂17c). */
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin_a, true, false);
        esp_rom_gpio_connect_out_signal((uint32_t)pin_a, PWM0_OUT0A_IDX, false, false);
        /* Input on as well: the verification reads the pin back. */
        gpio_ll_input_enable(&GPIO, (uint8_t)pin_a);
    }

    if (pin_b >= 0 && pin_b < SOC_GPIO_PIN_COUNT &&
        ((SOC_GPIO_VALID_GPIO_MASK >> pin_b) & 1ULL))
    {
        gpio_ll_func_sel(&GPIO, (uint8_t)pin_b, PIN_FUNC_GPIO);
        gpio_ll_output_enable(&GPIO, (uint8_t)pin_b);
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin_b, true, false);
        esp_rom_gpio_connect_out_signal((uint32_t)pin_b, PWM0_OUT0B_IDX, false, false);
        gpio_ll_input_enable(&GPIO, (uint8_t)pin_b);
    }

    return RT_EOK;
}

/* ------------------------------------------------------------------ msh -- */

/* `c5mcpwm [freq_hz] [duty_permille]`
 *
 * Two measurements, both from the output pad itself:
 *   - PCNT counts rising edges over a window, giving the frequency.
 *   - mcycle polling measures the high time relative to the period, giving duty.
 *
 * The counter is the independently verified one from 搂17d, so this is two drivers
 * checking each other rather than one timestamping itself.
 */
int c5_mcpwm_selftest(rt_uint32_t freq_hz, rt_uint32_t duty_permille)
{
    rt_uint32_t start;
    rt_uint32_t window_us;
    rt_uint32_t high_cycles = 0;
    rt_uint32_t period_cycles = 0;
    rt_uint32_t measured_hz;
    rt_uint32_t measured_permille;
    rt_uint32_t state;
    rt_uint32_t i;
    int edges;
    rt_err_t err;

    if (s_mcpwm == RT_NULL)
    {
        rt_kprintf("c5mcpwm : FAIL, MCPWM not initialised\n");
        return -1;
    }

    if (freq_hz == 0u || freq_hz > 1000000u)
    {
        freq_hz = 1000u;
    }
    if (duty_permille == 0u || duty_permille > 1000u)
    {
        duty_permille = 500u;
    }

    err = rt_hw_mcpwm_setup(freq_hz, duty_permille, C5_MCPWM_DEAD_TICKS);
    if (err != RT_EOK)
    {
        rt_kprintf("c5mcpwm : FAIL, setup rejected %u Hz (%d)\n",
                   (unsigned)freq_hz, (int)err);
        return -1;
    }

    rt_hw_mcpwm_set_pin(C5_MCPWM_TEST_PIN_A, C5_MCPWM_TEST_PIN_B);

    /* ---- 1. frequency, counted by PCNT ---- */

    if (rt_hw_pcnt_setup(0, C5_MCPWM_TEST_PIN_A) != RT_EOK)
    {
        rt_kprintf("c5mcpwm : FAIL, PCNT cannot watch IO%d\n",
                   (int)C5_MCPWM_TEST_PIN_A);
        return -1;
    }

    rt_hw_pcnt_clear(0);

    /* The window has to be chosen from the frequency, not fixed.
     *
     * A fixed 50 ms window quantises the result to 20 Hz steps (one edge per
     * 50 ms), so 50 Hz measured as 40 Hz - 2 edges instead of 2.5 - and failed a
     * 10 % check that the hardware passed easily. Measured exactly that.
     *
     * Aim for 20 edges, which puts the quantisation at 5 %: still inside the
     * tolerance, while at 1 kHz the window is clamped back to 50 ms so the shell
     * is not blocked for a second. The 500 ms ceiling keeps the worst case (very
     * low frequencies) bounded. */
    window_us = (20000000u / freq_hz);
    if (window_us < 50000u)
    {
        window_us = 50000u;
    }
    if (window_us > 500000u)
    {
        window_us = 500000u;
    }
    start = c5_mcpwm_cycles();
    while ((rt_uint32_t)(c5_mcpwm_cycles() - start) <
           (window_us * (rt_uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ))
    {
        /* spin */
    }

    edges = rt_hw_pcnt_get(0);
    measured_hz = (rt_uint32_t)edges * 1000000u / window_us;

    /* ---- 2. duty, by polling the pad ---- */

    /* Find a rising edge first, then time the high and low halves. Measuring from
     * an arbitrary starting level would divide by a partial period. */
    state = gpio_ll_get_level(&GPIO, (uint8_t)C5_MCPWM_TEST_PIN_A);
    start = c5_mcpwm_cycles();
    while (gpio_ll_get_level(&GPIO, (uint8_t)C5_MCPWM_TEST_PIN_A) != 0u)
    {
        if ((rt_uint32_t)(c5_mcpwm_cycles() - start) >
            (1000u * (rt_uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ))
        {
            break;      /* not toggling at all - reported below */
        }
    }
    while (gpio_ll_get_level(&GPIO, (uint8_t)C5_MCPWM_TEST_PIN_A) == 0u)
    {
        if ((rt_uint32_t)(c5_mcpwm_cycles() - start) >
            (1000u * (rt_uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ))
        {
            break;
        }
    }

    /* Average over a few periods: a single one is dominated by the polling loop's
     * own granularity. */
    for (i = 0; i < 8u; i++)
    {
        rt_uint32_t t_rise;
        rt_uint32_t t_fall;
        rt_uint32_t t_rise2;

        /* Rise to rise, NOT "start polling to fall".
         *
         * The first version captured t0 before waiting for the rising edge, so
         * the "period" it accumulated was (leftover low time) + (full high time)
         * rather than one period. Measured: 533/1000 for a requested 500, a 6.6 %
         * error that looked like a PWM defect and was entirely the measurement.
         *
         * A period is rise -> fall -> rise. */
        while (gpio_ll_get_level(&GPIO, (uint8_t)C5_MCPWM_TEST_PIN_A) != 0u)
        {
        }
        while (gpio_ll_get_level(&GPIO, (uint8_t)C5_MCPWM_TEST_PIN_A) == 0u)
        {
        }
        t_rise = c5_mcpwm_cycles();

        while (gpio_ll_get_level(&GPIO, (uint8_t)C5_MCPWM_TEST_PIN_A) != 0u)
        {
        }
        t_fall = c5_mcpwm_cycles();

        while (gpio_ll_get_level(&GPIO, (uint8_t)C5_MCPWM_TEST_PIN_A) == 0u)
        {
        }
        t_rise2 = c5_mcpwm_cycles();

        high_cycles   += (t_fall - t_rise);
        period_cycles += (t_rise2 - t_rise);
    }

    /* 64-bit intermediate, and it is not defensive coding.
     *
     * `high_cycles * 1000` overflows at 100 Hz: a period there is 480 000 CPU
     * cycles, so one accumulated period is ~19 200 000 and the product is
     * 4.8e9 - past 2^32. Measured: `c5mcpwm 100 250` reported a duty of 26/1000
     * and `c5mcpwm 100 900` reported 5/1000, both FAIL, while the same driver was
     * correct at 1 kHz and above because the product fitted. The hardware was
     * right every time; the arithmetic was not.
     *
     * Dividing first would lose resolution (1000/period truncates to 0 for any
     * period above 1000 cycles), so the widening is the correct fix rather than a
     * workaround. */
    measured_permille = period_cycles
                        ? (rt_uint32_t)(((rt_uint64_t)high_cycles * 1000u) /
                                        period_cycles) : 0u;

    rt_kprintf("c5mcpwm : %u Hz %u/1000 requested, dead time %u tick(s)\n",
               (unsigned)freq_hz, (unsigned)duty_permille,
               (unsigned)C5_MCPWM_DEAD_TICKS);
    rt_kprintf("c5mcpwm : frequency %u Hz from %d edge(s) in %u us (PCNT)\n",
               (unsigned)measured_hz, edges, (unsigned)window_us);
    rt_kprintf("c5mcpwm : duty %u/1000 from %u high / %u total cycle(s)\n",
               (unsigned)measured_permille, (unsigned)high_cycles,
               (unsigned)period_cycles);

    rt_hw_mcpwm_stop();

    /* Tolerance: the frequency window is quantised by whole edges in 50 ms (20 Hz
     * at 1 kHz), and the duty by the polling loop. 10 % covers both generously
     * while still catching a wrong prescaler, which shows up as an integer
     * multiple. */
    {
        rt_uint32_t hz_lo = freq_hz - (freq_hz / 10u);
        rt_uint32_t hz_hi = freq_hz + (freq_hz / 10u);
        rt_uint32_t du_lo = duty_permille - (duty_permille / 10u);
        rt_uint32_t du_hi = duty_permille + (duty_permille / 10u);

        if (edges == 0)
        {
            rt_kprintf("c5mcpwm : FAIL, no edges counted - nothing reached the "
                       "pad\n");
            return -1;
        }

        if (measured_hz < hz_lo || measured_hz > hz_hi)
        {
            rt_kprintf("c5mcpwm : FAIL, %u Hz is not within 10%% of %u\n",
                       (unsigned)measured_hz, (unsigned)freq_hz);
            return -1;
        }

        if (measured_permille < du_lo || measured_permille > du_hi)
        {
            rt_kprintf("c5mcpwm : FAIL, duty %u/1000 is not within 10%% of %u\n",
                       (unsigned)measured_permille, (unsigned)duty_permille);
            return -1;
        }
    }

    rt_kprintf("c5mcpwm : PASS (frequency and duty both measured at the pad; the "
               "analog shape and the dead-time nanoseconds are not)\n");
    return 0;
}

#endif /* SOC_MCPWM_SUPPORTED */




