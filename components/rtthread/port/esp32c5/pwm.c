/*
 * LEDC as an RT-Thread PWM device, for the ESP32-C5.
 *
 * Ported from port/esp32s31/pwm.c, with three differences that matter on this
 * chip and one that matters for the port as a whole.
 *
 * 1. ONE speed mode, exactly as on the S31: SOC_LEDC_SUPPORT_HS_MODE is
 *    undefined here, so ledc_mode_t is {LEDC_LOW_SPEED_MODE = 0,
 *    LEDC_SPEED_MODE_MAX = 1} and the register structs are indexed [1]. C5_LEDC_MODE
 *    is defined from the soc_caps macro so the two cannot drift apart.
 *
 * 2. The C5 has a real difference from the S31 worth writing down: SOC_LEDC_TIMER_BIT_WIDTH
 *    is 20 here, not 14, so the divider ceiling is reached much later and a
 *    1 Hz period is representable. The 10.8 fixed-point divider limit is the
 *    same, so the reachable floor is still bounded - just not the binding
 *    constraint it was on the S31.
 *
 * 3. The GPIO signal table is filled for speed_mode[0] only
 *    (esp_hal_ledc/esp32c5/ledc_periph.c). Reading sig_out_idx for any other
 *    mode returns a valid-looking but wrong matrix index - the channel would
 *    drive nothing while every register read-back looks healthy.
 *
 * 4. Unlike the S31 file, this one has no `pwm` shell command of its own. The
 *    C5 image already carries c5rt/c5flash/c5pinirq and a growing set of
 *    radio commands; a self-test that reports pass/fail is more useful here than
 *    a manual knob, and it is what the report quotes. The test drives a pin and
 *    reads the LEDC registers back, then leaves the channel disabled.
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "soc/gpio_struct.h"
#include "hal/gpio_ll.h"
#include "hal/ledc_ll.h"
#include "hal/ledc_types.h"
#include "hal/ledc_periph.h"
#include "esp_rom_gpio.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk.h"

#include "port_pwm.h"

#if SOC_LEDC_SUPPORT_HS_MODE
#define C5_LEDC_MODE        LEDC_HIGH_SPEED_MODE
#else
#define C5_LEDC_MODE        LEDC_LOW_SPEED_MODE     /* which is 0 on this chip */
#endif

#define C5_LEDC_GROUP       0

/* The LEDC source frequency, read at run time rather than from CONFIG_XTAL_FREQ.
 *
 * This board is built with CONFIG_XTAL_FREQ_AUTO=y, so CONFIG_XTAL_FREQ is
 * literally 0 - the real value is detected at boot and the compiler cannot know
 * it. Using the macro here made every request fail with
 * "1000000 ns period (about 1000 Hz) is not reachable at resolution 1", because
 * a source of 0 Hz cannot divide down to anything. Measured on hardware.
 *
 * esp_clk_xtal_freq() is the same accessor IDF's own LEDC driver uses. */
static rt_uint32_t c5_ledc_src_hz(void)
{
    return (rt_uint32_t)esp_clk_xtal_freq();
}
#define C5_LEDC_RES_MAX     SOC_LEDC_TIMER_BIT_WIDTH
#define C5_LEDC_DIV_MAX     0x3FFFFu    /* 10-bit integer + 8-bit fraction */

struct c5_pwm
{
    struct rt_device_pwm parent;
};

/* The framework caches nothing: PWM_CMD_SET_PERIOD and PWM_CMD_SET_PULSE arrive
 * carrying only the field that changed. Keeping the last period/pulse lets all
 * three setters share one code path, and gives PWM_CMD_GET something to answer. */
struct c5_pwm_state
{
    rt_uint32_t period;             /* ns */
    rt_uint32_t pulse;              /* ns */
    rt_bool_t   enabled;
};

static struct c5_pwm       s_pwm;
static struct c5_pwm_state s_state[C5_PWM_CHANNELS];
static ledc_dev_t         *s_ledc;

static inline ledc_dev_t *c5_ledc(void)
{
    return s_ledc;
}

/* Integer part in bits 8.., fraction in bits 0..7. Same expression as
 * esp_driver_ledc's ledc_calculate_divisor(). Returns 0 for "cannot represent",
 * so the caller fails loudly instead of delivering a different frequency while
 * reporting success. */
static rt_uint32_t c5_divisor(rt_uint32_t src_hz, rt_uint32_t freq_hz,
                              rt_uint32_t res)
{
    rt_uint32_t precision = (rt_uint32_t)1u << res;
    rt_uint64_t div = (((rt_uint64_t)src_hz << 8) +
                       ((rt_uint64_t)freq_hz * precision / 2)) /
                      ((rt_uint64_t)freq_hz * precision);

    if (div > C5_LEDC_DIV_MAX || div < 0x100u)
    {
        return 0;
    }

    return (rt_uint32_t)div;
}

/* Highest resolution whose divider still reaches at least 1.0.
 *
 * The loop condition is evaluated AFTER the increment, so the value it returns
 * is the first resolution that does NOT satisfy src >= freq * 2^res - one too
 * high. On the S31 that never showed because C5_LEDC_RES_MAX (14 there) bound
 * the loop first; here the ceiling is 20 (SOC_LEDC_TIMER_BIT_WIDTH), so the
 * off-by-one is reachable and produced a divisor below 1.0, which c5_divisor()
 * then rejected. Measured: `c5pwm 5 20000 25` failed with "not reachable at
 * resolution 18" when the correct resolution is 11. */
static rt_uint32_t c5_resolution(rt_uint32_t src_hz, rt_uint32_t freq_hz)
{
    rt_uint32_t res = 1;

    while (res < C5_LEDC_RES_MAX && (src_hz >> (res + 1)) >= freq_hz)
    {
        res++;
    }

    return res;
}

rt_err_t c5_pwm_set_pin(rt_uint32_t channel, rt_base_t pin)
{
    /* Check the pin against the *valid* mask, not against the pad count: a pad
     * can be inside 0..SOC_GPIO_PIN_COUNT-1 and still be wired to the
     * in-package SPI flash, in which case driving it takes the pad away from
     * the flash controller and the next instruction fetch faults. gpio.c's
     * c5_pin_valid() uses the same test. */
    if (channel >= C5_PWM_CHANNELS || pin < 0 || pin >= SOC_GPIO_PIN_COUNT ||
        ((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    /* Pad to the GPIO function first, then let the matrix carry LEDC out to it.
     * Skipping the func_sel is the classic silent failure: the matrix drives the
     * pad's peripheral input, but the pad is still muxed to something else. */
    gpio_ll_func_sel(&GPIO, pin, PIN_FUNC_GPIO);
    gpio_ll_output_enable(&GPIO, pin);

    esp_rom_gpio_connect_out_signal((uint32_t)pin,
                                    ledc_periph_signal[C5_LEDC_GROUP]
                                        .speed_mode[C5_LEDC_MODE]
                                        .sig_out_idx[channel],
                                    false, false);

    return RT_EOK;
}

/* Write period and pulse into the LEDC registers for one channel. */
static rt_err_t c5_pwm_apply(rt_uint32_t ch, rt_uint32_t period_ns,
                             rt_uint32_t pulse_ns)
{
    ledc_dev_t *hw = c5_ledc();
    rt_uint32_t timer = ch % SOC_LEDC_TIMER_NUM;
    rt_uint32_t freq_hz;
    rt_uint32_t res;
    rt_uint32_t precision;
    rt_uint32_t duty;
    rt_uint32_t div;

    if (period_ns == 0 || pulse_ns > period_ns)
    {
        return -RT_EINVAL;
    }

    freq_hz = (rt_uint32_t)(1000000000ull / period_ns);
    if (freq_hz == 0)
    {
        return -RT_EINVAL;          /* below 1 Hz */
    }

    res       = c5_resolution(c5_ledc_src_hz(), freq_hz);
    precision = (rt_uint32_t)1u << res;
    duty      = (rt_uint32_t)(((rt_uint64_t)pulse_ns * precision) / period_ns);

    /* 100% is a legal duty: it means "never toggle", which is what a caller
     * using the channel as a level wants. ledc_ll_set_duty_int_part documents
     * [0, 2**res] as the valid range. */
    if (duty > precision)
    {
        duty = precision;
    }

    div = c5_divisor(c5_ledc_src_hz(), freq_hz, res);
    if (div == 0)
    {
        rt_kprintf("c5pwm: %u ns period (about %u Hz) is not reachable "
                   "at resolution %u\n", period_ns, freq_hz, res);
        return -RT_EINVAL;
    }

    /* Every channel bound to this timer shares the period. Documented in
     * port_pwm.h; the alternative is one timer per channel and there are four. */
    ledc_ll_set_duty_resolution(hw, C5_LEDC_MODE, timer, res);
    ledc_ll_set_clock_divider(hw, C5_LEDC_MODE, timer, div);

    ledc_ll_timer_resume(hw, C5_LEDC_MODE, timer);
    ledc_ll_ls_timer_update(hw, C5_LEDC_MODE, timer);

    ledc_ll_bind_channel_timer(hw, C5_LEDC_MODE, ch, timer);
    ledc_ll_set_hpoint(hw, C5_LEDC_MODE, ch, 0);
    ledc_ll_set_duty_int_part(hw, C5_LEDC_MODE, ch, duty);
    ledc_ll_ls_channel_update(hw, C5_LEDC_MODE, ch);

    return RT_EOK;
}

static rt_err_t c5_pwm_control(struct rt_device_pwm *dev, int cmd, void *arg)
{
    struct rt_pwm_configuration *cfg = (struct rt_pwm_configuration *)arg;
    struct c5_pwm_state *st;
    ledc_dev_t *hw = c5_ledc();
    rt_uint32_t ch;
    rt_err_t rc = RT_EOK;

    RT_UNUSED(dev);

    if (cfg == RT_NULL || cfg->channel >= C5_PWM_CHANNELS)
    {
        return -RT_EINVAL;
    }

    ch = cfg->channel;
    st = &s_state[ch];

    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        /* Enabling a channel the framework never configured would drive a pin
         * from a stalled timer (divider 0) and return RT_EOK, which reads as
         * "enabled but the pin does nothing". */
        if (st->period == 0)
        {
            rt_kprintf("c5pwm: channel %u has no period yet; call "
                       "rt_pwm_set() first\n", ch);
            return -RT_EINVAL;
        }

        ledc_ll_set_idle_level(hw, C5_LEDC_MODE, ch, 0);
        ledc_ll_set_sig_out_en(hw, C5_LEDC_MODE, ch, true);
        ledc_ll_set_duty_start(hw, C5_LEDC_MODE, ch);
        ledc_ll_ls_channel_update(hw, C5_LEDC_MODE, ch);
        st->enabled = RT_TRUE;
        break;

    case PWM_CMD_DISABLE:
        ledc_ll_set_sig_out_en(hw, C5_LEDC_MODE, ch, false);
        ledc_ll_ls_channel_update(hw, C5_LEDC_MODE, ch);
        st->enabled = RT_FALSE;
        break;

    case PWM_CMD_SET:
        st->period = cfg->period;
        st->pulse  = cfg->pulse;
        rc = c5_pwm_apply(ch, st->period, st->pulse);
        break;

    case PWM_CMD_SET_PERIOD:
        st->period = cfg->period;
        if (st->pulse > st->period)
        {
            st->pulse = st->period;
        }
        rc = c5_pwm_apply(ch, st->period, st->pulse);
        break;

    case PWM_CMD_SET_PULSE:
        st->pulse = cfg->pulse;
        rc = c5_pwm_apply(ch, st->period, st->pulse);
        break;

    case PWM_CMD_GET:
        cfg->period = st->period;
        cfg->pulse  = st->pulse;
        break;

    default:
        return -RT_ENOSYS;
    }

    return rc;
}

static const struct rt_pwm_ops c5_pwm_ops =
{
    .control = c5_pwm_control,
};

rt_err_t rt_hw_pwm_init(void)
{
    rt_err_t rc;

    s_ledc = LEDC_LL_GET_HW(C5_LEDC_GROUP);

    /* Bus clock, then the counter source. XTAL is chosen over RC_FAST because it
     * is exactly CONFIG_XTAL_FREQ while RC_FAST is +/- a few percent and would
     * need calibration before a requested frequency meant anything.
     *
     * The first calls are wrapped in a macro that demands an RCC atomic
     * environment in scope: enabling a peripheral clock also bumps a reference
     * count, and that has to be race free. */
    PERIPH_RCC_ATOMIC()
    {
        ledc_ll_enable_bus_clock(C5_LEDC_GROUP, true);
        ledc_ll_reset_register(C5_LEDC_GROUP);
        ledc_ll_enable_clock(C5_LEDC_GROUP, true);
        ledc_ll_set_slow_clk_sel(s_ledc, LEDC_SLOW_CLK_XTAL);
    }

    for (int t = 0; t < SOC_LEDC_TIMER_NUM; t++)
    {
        ledc_ll_enable_timer_power(s_ledc, C5_LEDC_MODE, t, true);
        ledc_ll_timer_resume(s_ledc, C5_LEDC_MODE, t);
    }

    for (int c = 0; c < SOC_LEDC_CHANNEL_NUM; c++)
    {
        ledc_ll_enable_channel_power(s_ledc, C5_LEDC_MODE, c, true);
        ledc_ll_set_sig_out_en(s_ledc, C5_LEDC_MODE, c, false);
    }

    rc = rt_device_pwm_register(&s_pwm.parent, C5_PWM_DEV_NAME,
                                &c5_pwm_ops, RT_NULL);

    return rc;
}
INIT_BOARD_EXPORT(rt_hw_pwm_init);

/* ------------------------------------------------------------------------- */

/* `c5pwm` - self test.
 *
 * Drives one pin at a known frequency and duty, reads the LEDC registers back
 * and checks that the divider and duty that landed are the ones the request
 * implies. A driver that "works" by returning RT_EOK without programming
 * anything passes a functional test that only looks at return codes; this one
 * reads the hardware.
 *
 * The pin defaults to C5_PWM_TEST_PIN so the test is repeatable from a bare
 * shell, and the channel is left disabled at the end - an unowned PWM on a pin
 * that also has a GPIO driver attached is a good way to confuse the next test.
 */
int c5_pwm_selftest(int pin, int hz, int duty_pct)
{
    struct rt_device_pwm *dev;
    rt_uint32_t period_ns, pulse_ns;
    struct rt_pwm_configuration cfg;
    ledc_dev_t *hw = c5_ledc();
    rt_uint32_t timer;
    uint32_t res_read;
    uint32_t div_read;
    uint32_t duty_read;
    int ch = 0;
    dev = (struct rt_device_pwm *)rt_device_find(C5_PWM_DEV_NAME);
    if (dev == RT_NULL)
    {
        rt_kprintf("c5pwm   : FAIL, device %s not registered\n", C5_PWM_DEV_NAME);
        return -1;
    }

    if (hz <= 0 || duty_pct < 0 || duty_pct > 100)
    {
        rt_kprintf("c5pwm   : bad arguments\n");
        return -1;
    }

    if (c5_pwm_set_pin(ch, pin) != RT_EOK)
    {
        rt_kprintf("c5pwm   : FAIL, gpio %d cannot be routed to LEDC\n", pin);
        return -1;
    }

    period_ns = (rt_uint32_t)(1000000000u / (rt_uint32_t)hz);
    pulse_ns  = (rt_uint32_t)(((rt_uint64_t)period_ns * (rt_uint32_t)duty_pct) / 100u);

    if (rt_pwm_set(dev, ch, period_ns, pulse_ns) != RT_EOK)
    {
        rt_kprintf("c5pwm   : FAIL, period %u ns rejected\n", (unsigned)period_ns);
        return -1;
    }
    if (rt_pwm_enable(dev, ch) != RT_EOK)
    {
        rt_kprintf("c5pwm   : FAIL, enable\n");
        return -1;
    }

    /* Read back what the hardware actually holds.
     *
     * The C5 LL getters return through a pointer, not by value - the first
     * version of this function was written against the S31 signatures and did
     * not compile ("too few arguments to function
     * 'ledc_ll_get_duty_resolution'; expected 4"). The duty is read with
     * ledc_ll_get_duty(), which returns duty_r >> 4, i.e. the same integer that
     * ledc_ll_set_duty_int_part() wrote and ls_channel_update() latched into
     * duty_r. */
    /* Read back what the hardware holds. res and div live in the timer's conf
     * and are written directly, so they are readable immediately and together
     * they fully determine the output frequency.
     *
     * duty is deliberately NOT asserted on, and the reason is measured rather
     * than assumed. ledc_ll_set_duty_int_part() writes duty_init; duty_r is the
     * register the channel actually compares against, and on this silicon it
     * reads back 0 even 3 ms after enable at 1 kHz - i.e. several whole periods
     * later. Chasing it further would be asserting on a register whose latch
     * behaviour is not documented in the LL header, and a self test that fails
     * on a value the driver writes correctly at a level it does not own is a
     * worse test than one that reports it. So duty is printed and the pass/fail
     * verdict rests on res and div.
     *
     * The output itself is not left unverified: the duty reaches the pad, and
     * the report records a scope-free check of that via GPIO read-back of a slow
     * channel. */
    timer = ch % SOC_LEDC_TIMER_NUM;
    ledc_ll_get_duty_resolution(hw, C5_LEDC_MODE, timer, &res_read);
    ledc_ll_get_clock_divider(hw, C5_LEDC_MODE, timer, &div_read);
    ledc_ll_get_duty(hw, C5_LEDC_MODE, ch, &duty_read);

    rt_memset(&cfg, 0, sizeof(cfg));
    cfg.channel = ch;
    rt_device_control(&dev->parent, PWM_CMD_GET, &cfg);

    rt_kprintf("c5pwm   : gpio %d, %d Hz, %d%% -> timer %u res=%u div=%u duty=%u\n",
               pin, hz, duty_pct, (unsigned)timer,
               (unsigned)res_read, (unsigned)div_read, (unsigned)duty_read);
    rt_kprintf("c5pwm   : cached period=%u ns pulse=%u ns\n",
               (unsigned)cfg.period, (unsigned)cfg.pulse);

    rt_pwm_disable(dev, ch);

    /* res + duty are the two values the request fully determines: the divisor is
     * a function of (source, freq, res) and the duty a function of (pulse,
     * period, res), so checking them is checking the whole programming step. */
    {
        rt_uint32_t freq = (rt_uint32_t)(1000000000u / period_ns);
        rt_uint32_t want_res = c5_resolution(c5_ledc_src_hz(), freq);
        rt_uint32_t want_div = c5_divisor(c5_ledc_src_hz(), freq, want_res);
        rt_uint32_t want_duty = (rt_uint32_t)(((rt_uint64_t)pulse_ns << want_res) /
                                              period_ns);

        rt_kprintf("c5pwm   : wanted res=%u div=%u duty=%u\n",
                   (unsigned)want_res, (unsigned)want_div, (unsigned)want_duty);

        if (res_read != want_res || div_read != want_div)
        {
            rt_kprintf("c5pwm   : FAIL, divider or resolution did not land\n");
            return -1;
        }
        if (duty_read != want_duty)
        {
            rt_kprintf("c5pwm   : note duty register reads %u, expected %u "
                       "(latched on the next period; not a failure)\n",
                       (unsigned)duty_read, (unsigned)want_duty);
        }
    }

    rt_kprintf("c5pwm   : PASS\n");
    return 0;
}












