/*
 * LEDC as an RT-Thread PWM device.
 *
 * Why this is worth having: PWM is the one peripheral where RT-Thread already
 * defines a device class (components/drivers/misc/rt_drv_pwm.c) with a
 * nanoseconds-based interface, so there is no invented API here - rt_pwm_set(),
 * rt_pwm_enable() and the `pwm0` device behave as on any other BSP.
 *
 * Three things about the S31 silicon drive the shape of this file.
 *
 * 1. There is exactly ONE speed mode. SOC_LEDC_SUPPORT_HS_MODE is undefined, so
 *    ledc_mode_t is just {LEDC_LOW_SPEED_MODE = 0, LEDC_SPEED_MODE_MAX = 1} and
 *    the register structs are indexed [1]. S31 does have two LEDC instances
 *    (LEDC_LL_GROUP_NUM == 2), but those are separate register blocks reached
 *    through LEDC_LL_GET_HW(group), not "speed modes". Passing 1 where a speed
 *    mode is expected runs off the end of those arrays.
 *    Everything below goes through S31_LEDC_MODE, which is defined from
 *    SOC_LEDC_SUPPORT_HS_MODE so the mistake cannot be made again.
 *
 * 2. The GPIO signal table only fills speed_mode[0]
 *    (esp_hal_ledc/esp32s31/ledc_periph.c), for both groups. Reading
 *    sig_out_idx for any other mode returns zero, which is a valid-looking but
 *    wrong matrix index - the channel would then drive nothing while every
 *    register read-back looks healthy.
 *
 * 3. The clock divider is a fixed-point value: integer part << 8, fraction in
 *    the low 8 bits (LEDC_LL_FRACTIONAL_BITS). The formula below is the same one
 *    esp_driver_ledc uses so a given period produces the same divider, and the
 *    division is done in 64-bit to keep the rounding term from overflowing at
 *    high resolution.
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

#include "port_pwm.h"

#if SOC_LEDC_SUPPORT_HS_MODE
#define S31_LEDC_MODE       LEDC_HIGH_SPEED_MODE
#else
#define S31_LEDC_MODE       LEDC_LOW_SPEED_MODE     /* which is 0 on this chip */
#endif

#define S31_LEDC_GROUP      0
#define S31_LEDC_SRC_HZ     (CONFIG_XTAL_FREQ * 1000000u)
#define S31_LEDC_RES_MAX    14          /* 16384 steps */
#define S31_LEDC_DIV_MAX    0x3FFFFu    /* 10-bit integer + 8-bit fraction */

struct s31_pwm
{
    struct rt_device_pwm parent;
};

/* The framework caches nothing: PWM_CMD_SET_PERIOD and PWM_CMD_SET_PULSE arrive
 * carrying only the field that changed. Keeping the last period/pulse lets all
 * three setters share one code path, and gives PWM_CMD_GET something to answer
 * with. */
struct s31_pwm_state
{
    rt_uint32_t period;             /* ns */
    rt_uint32_t pulse;              /* ns */
    rt_bool_t   enabled;
};

static struct s31_pwm       s_pwm;
static struct s31_pwm_state s_state[S31_PWM_CHANNELS];
static ledc_dev_t          *s_ledc;

static inline ledc_dev_t *s31_ledc(void)
{
    return s_ledc;
}

/* Integer part in bits 8.., fraction in bits 0..7. Same expression as
 * esp_driver_ledc's ledc_calculate_divisor(). */
static rt_uint32_t s31_divisor(rt_uint32_t src_hz, rt_uint32_t freq_hz,
                               rt_uint32_t res)
{
    rt_uint32_t precision = (rt_uint32_t)1u << res;
    rt_uint64_t div = (((rt_uint64_t)src_hz << 8) +
                       ((rt_uint64_t)freq_hz * precision / 2)) /
                      ((rt_uint64_t)freq_hz * precision);

    if (div > S31_LEDC_DIV_MAX)
    {
        div = S31_LEDC_DIV_MAX;
    }
    if (div < 0x100u)               /* below 1.0 the timer would stall */
    {
        div = 0x100u;
    }

    return (rt_uint32_t)div;
}

/* Highest resolution whose divider still reaches at least 1.0. */
static rt_uint32_t s31_resolution(rt_uint32_t src_hz, rt_uint32_t freq_hz)
{
    rt_uint32_t res = 1;

    while (res < S31_LEDC_RES_MAX && (src_hz >> res) >= freq_hz)
    {
        res++;
    }

    return res;
}

rt_err_t s31_pwm_set_pin(rt_uint32_t channel, rt_base_t pin)
{
    if (channel >= S31_PWM_CHANNELS || pin < 0 || pin >= SOC_GPIO_PIN_COUNT)
    {
        return -RT_EINVAL;
    }

    /* Pad to the GPIO function first, then let the matrix carry LEDC out to it.
     * Skipping the func_sel is the classic silent failure: the matrix drives the
     * pad's peripheral input, but the pad is still muxed to something else. */
    gpio_ll_func_sel(&GPIO, pin, PIN_FUNC_GPIO);
    gpio_ll_output_enable(&GPIO, pin);

    esp_rom_gpio_connect_out_signal((uint32_t)pin,
                                    ledc_periph_signal[S31_LEDC_GROUP]
                                        .speed_mode[S31_LEDC_MODE]
                                        .sig_out_idx[channel],
                                    false, false);

    return RT_EOK;
}

/* Write period and pulse into the LEDC registers for one channel. */
static rt_err_t s31_pwm_apply(rt_uint32_t ch, rt_uint32_t period_ns,
                              rt_uint32_t pulse_ns)
{
    ledc_dev_t *hw = s31_ledc();
    rt_uint32_t timer = ch % SOC_LEDC_TIMER_NUM;
    rt_uint32_t freq_hz;
    rt_uint32_t res;
    rt_uint32_t precision;
    rt_uint32_t duty;

    if (period_ns == 0 || pulse_ns > period_ns)
    {
        return -RT_EINVAL;
    }

    freq_hz = (rt_uint32_t)(1000000000ull / period_ns);
    if (freq_hz == 0)
    {
        return -RT_EINVAL;          /* below 1 Hz */
    }

    res       = s31_resolution(S31_LEDC_SRC_HZ, freq_hz);
    precision = (rt_uint32_t)1u << res;
    duty      = (rt_uint32_t)(((rt_uint64_t)pulse_ns * precision) / period_ns);

    if (duty >= precision)
    {
        duty = precision - 1;       /* 100% would never toggle the pin */
    }

    /* Every channel bound to this timer shares the period. Documented in
     * port_pwm.h; the alternative is one timer per channel and there are
     * only four. */
    ledc_ll_set_duty_resolution(hw, S31_LEDC_MODE, timer, res);
    ledc_ll_set_clock_divider(hw, S31_LEDC_MODE, timer,
                              s31_divisor(S31_LEDC_SRC_HZ, freq_hz, res));
    ledc_ll_timer_resume(hw, S31_LEDC_MODE, timer);
    ledc_ll_ls_timer_update(hw, S31_LEDC_MODE, timer);

    ledc_ll_bind_channel_timer(hw, S31_LEDC_MODE, ch, timer);
    ledc_ll_set_hpoint(hw, S31_LEDC_MODE, ch, 0);
    ledc_ll_set_duty_int_part(hw, S31_LEDC_MODE, ch, duty);
    ledc_ll_ls_channel_update(hw, S31_LEDC_MODE, ch);

    return RT_EOK;
}

static rt_err_t s31_pwm_control(struct rt_device_pwm *dev, int cmd, void *arg)
{
    struct rt_pwm_configuration *cfg = (struct rt_pwm_configuration *)arg;
    struct s31_pwm_state *st;
    ledc_dev_t *hw = s31_ledc();
    rt_uint32_t ch;
    rt_err_t rc = RT_EOK;

    RT_UNUSED(dev);

    if (cfg == RT_NULL || cfg->channel >= S31_PWM_CHANNELS)
    {
        return -RT_EINVAL;
    }

    ch = cfg->channel;
    st = &s_state[ch];

    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        ledc_ll_set_idle_level(hw, S31_LEDC_MODE, ch, 0);
        ledc_ll_set_sig_out_en(hw, S31_LEDC_MODE, ch, true);
        ledc_ll_set_duty_start(hw, S31_LEDC_MODE, ch);
        ledc_ll_ls_channel_update(hw, S31_LEDC_MODE, ch);
        st->enabled = RT_TRUE;
        break;

    case PWM_CMD_DISABLE:
        ledc_ll_set_sig_out_en(hw, S31_LEDC_MODE, ch, false);
        ledc_ll_ls_channel_update(hw, S31_LEDC_MODE, ch);
        st->enabled = RT_FALSE;
        break;

    case PWM_CMD_SET:
        st->period = cfg->period;
        st->pulse  = cfg->pulse;
        rc = s31_pwm_apply(ch, st->period, st->pulse);
        break;

    case PWM_CMD_SET_PERIOD:
        st->period = cfg->period;
        if (st->pulse > st->period)
        {
            st->pulse = st->period;
        }
        rc = s31_pwm_apply(ch, st->period, st->pulse);
        break;

    case PWM_CMD_SET_PULSE:
        st->pulse = cfg->pulse;
        rc = s31_pwm_apply(ch, st->period, st->pulse);
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

static const struct rt_pwm_ops s31_pwm_ops =
{
    .control = s31_pwm_control,
};

rt_err_t rt_hw_pwm_init(void)
{
    rt_err_t rc;

    s_ledc = LEDC_LL_GET_HW(S31_LEDC_GROUP);

    /* Bus clock, then the counter source. XTAL is chosen over RC_FAST because
     * it is exactly CONFIG_XTAL_FREQ while RC_FAST is +/- a few percent and
     * would need calibration before a requested frequency meant anything.
     *
     * The first calls are wrapped in a macro that demands an RCC atomic
     * environment in scope: enabling a peripheral clock also bumps a reference
     * count, and that has to be race free. */
    PERIPH_RCC_ATOMIC()
    {
        ledc_ll_enable_bus_clock(S31_LEDC_GROUP, true);
        ledc_ll_reset_register(S31_LEDC_GROUP);
        ledc_ll_enable_clock(S31_LEDC_GROUP, true);
        ledc_ll_set_slow_clk_sel(s_ledc, LEDC_SLOW_CLK_XTAL);
    }

    for (int t = 0; t < SOC_LEDC_TIMER_NUM; t++)
    {
        ledc_ll_enable_timer_power(s_ledc, S31_LEDC_MODE, t, true);
        ledc_ll_timer_resume(s_ledc, S31_LEDC_MODE, t);
    }

    for (int c = 0; c < SOC_LEDC_CHANNEL_NUM; c++)
    {
        ledc_ll_enable_channel_power(s_ledc, S31_LEDC_MODE, c, true);
        ledc_ll_set_sig_out_en(s_ledc, S31_LEDC_MODE, c, false);
    }

    rc = rt_device_pwm_register(&s_pwm.parent, S31_PWM_DEV_NAME,
                                &s31_pwm_ops, RT_NULL);

    return rc;
}

/* ------------------------------------------------------------------------- */

static int cmd_pwm(int argc, char **argv)
{
    struct rt_device_pwm *dev =
        (struct rt_device_pwm *)rt_device_find(S31_PWM_DEV_NAME);
    rt_uint32_t ch, hz, duty;
    rt_uint32_t period_ns, pulse_ns;
    rt_base_t pin;

    if (dev == RT_NULL)
    {
        rt_kprintf("s31pwm: device %s not registered\n", S31_PWM_DEV_NAME);
        return -1;
    }

    if (argc == 3 && rt_strcmp(argv[1], "off") == 0)
    {
        ch = (rt_uint32_t)atoi(argv[2]);
        if (rt_pwm_disable(dev, (int)ch) != RT_EOK)
        {
            rt_kprintf("s31pwm: channel %d not configured\n", (int)ch);
            return -1;
        }
        rt_kprintf("s31pwm: channel %d off\n", (int)ch);
        return 0;
    }

    if (argc != 5)
    {
        rt_kprintf("usage: s31pwm <ch 0..%d> <gpio> <hz> <duty %%>\n",
                   S31_PWM_CHANNELS - 1);
        rt_kprintf("       s31pwm off <ch>\n");
        return -1;
    }

    ch   = (rt_uint32_t)atoi(argv[1]);
    pin  = (rt_base_t)atoi(argv[2]);
    hz   = (rt_uint32_t)atoi(argv[3]);
    duty = (rt_uint32_t)atoi(argv[4]);

    if (ch >= S31_PWM_CHANNELS || hz == 0 || duty > 100)
    {
        rt_kprintf("s31pwm: bad arguments\n");
        return -1;
    }

    if (s31_pwm_set_pin(ch, pin) != RT_EOK)
    {
        rt_kprintf("s31pwm: gpio %d cannot be routed\n", (int)pin);
        return -1;
    }

    period_ns = 1000000000u / hz;
    pulse_ns  = (rt_uint32_t)(((rt_uint64_t)period_ns * duty) / 100u);

    if (rt_pwm_set(dev, (int)ch, period_ns, pulse_ns) != RT_EOK)
    {
        rt_kprintf("s31pwm: period %d ns rejected\n", (int)period_ns);
        return -1;
    }
    if (rt_pwm_enable(dev, (int)ch) != RT_EOK)
    {
        rt_kprintf("s31pwm: enable failed\n");
        return -1;
    }

    rt_kprintf("s31pwm: gpio %d, %d Hz, %d%%, timer %d (period %d ns, pulse %d ns)\n",
               (int)pin, (int)hz, (int)duty, (int)(ch % SOC_LEDC_TIMER_NUM),
               (int)period_ns, (int)pulse_ns);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_pwm, s31pwm, LEDC PWM: s31pwm <ch> <gpio> <hz> <duty%>);
