/*
 * ESP32-C5 PCNT (pulse counter), registered as an RT-Thread pulse-encoder device.
 *
 * RT-Thread ships components/drivers/misc/pulse_encoder.c and
 * include/drivers/pulse_encoder.h behind RT_USING_PULSE_ENCODER, so the count is
 * reachable the RT-Thread way:
 *
 *     rt_device_t d = rt_device_find("pcnt0");
 *     rt_int32_t count;
 *     rt_device_open(d, RT_DEVICE_OFLAG_RDONLY);
 *     rt_device_read(d, 0, &count, 1);
 *     rt_device_control(d, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
 *
 * ---------------------------------------------------------------------------
 * How this is verified without any external wiring.
 *
 * A PCNT unit counts edges seen on a pad, and the pad's input path sees whatever
 * the pad is driven to - including by its own output driver. So the test drives a
 * GPIO high and low from software N times and lets PCNT count the edges on that
 * same pin. That is a genuine round trip through the input matrix, the glitch
 * filter and the counter; it just does not need a jumper. The expected count is
 * exactly N rising edges, so this is a real pass/fail rather than "something
 * moved".
 *
 * What it does NOT verify: an external driver's edge shape, and any frequency
 * high enough for the glitch filter to matter.
 *
 * ---------------------------------------------------------------------------
 * C5 specifics, versus the S31 file this was ported from:
 *
 *  - ONE group (PCNT), four units, two channels per unit. `PCNT_LL_GET_HW()`
 *    returns NULL for anything but 0.
 *
 *  - The pad signals are named PCNT_SIG_CH0_IN<n> / PCNT_CTRL_CH0_IN<n> /
 *    PCNT_RST_IN<n> and are still channel-major in the header (all CH0 entries,
 *    then CH1), but the per-unit indices are NOT contiguous: unit 0 is 101/103,
 *    unit 1 is 105/107, unit 2 is 109/111, unit 3 is 113/115. Indexing this table
 *    arithmetically instead of reading it picks a different unit's channel.
 *
 *  - The clock source is APB and only APB - pcnt_ll_set_clock_source() HAL_ASSERTs
 *    anything else. That is unlike the ADC/CAN/RMT pattern in this port, where
 *    the default source had to be switched away from the disabled PLL.
 *
 *  - The counter is 16-bit signed and the read helper sign-extends it. A count
 *    that "wraps" at 32767 is not a bug in this driver.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/pcnt_struct.h"
#include "hal/gpio_ll.h"
#include "hal/pcnt_ll.h"
#include "hal/pcnt_types.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_PCNT_SUPPORTED)

#include "port_pcnt.h"

#define C5_PCNT_UNITS   4

/* Signal indices for group 0, indexed [unit][channel]. Spelled out rather than
 * computed: the header's numbering is not contiguous per unit (see the note at
 * the top), and an off-by-one here silently counts a different pin. */
static int c5_pcnt_sig_idx(int unit, int channel)
{
    static const int idx[C5_PCNT_UNITS][2] =
    {
        { PCNT_SIG_CH0_IN0_IDX, PCNT_SIG_CH1_IN0_IDX },
        { PCNT_SIG_CH0_IN1_IDX, PCNT_SIG_CH1_IN1_IDX },
        { PCNT_SIG_CH0_IN2_IDX, PCNT_SIG_CH1_IN2_IDX },
        { PCNT_SIG_CH0_IN3_IDX, PCNT_SIG_CH1_IN3_IDX },
    };

    if (unit < 0 || unit >= C5_PCNT_UNITS || channel < 0 || channel > 1)
    {
        return -1;
    }

    return idx[unit][channel];
}

static struct rt_pulse_encoder_device s_pe_dev;

static int c5_pcnt_read_unit(int unit)
{
    if (unit < 0 || unit >= C5_PCNT_UNITS)
    {
        return 0;
    }

    return pcnt_ll_get_count(&PCNT, unit);
}

static void c5_pcnt_clear_unit(int unit)
{
    if (unit >= 0 && unit < C5_PCNT_UNITS)
    {
        pcnt_ll_clear_count(&PCNT, unit);
    }
}

static rt_err_t c5_pe_init(struct rt_pulse_encoder_device *pe)
{
    RT_UNUSED(pe);

    /* Nothing to do: rt_hw_pcnt_init() runs from the board init band and has
     * already enabled the group clock and selected the APB source. */
    return RT_EOK;
}

static rt_int32_t c5_pe_get_count(struct rt_pulse_encoder_device *pe)
{
    RT_UNUSED(pe);

    return (rt_int32_t)c5_pcnt_read_unit(0);
}

static rt_err_t c5_pe_clear_count(struct rt_pulse_encoder_device *pe)
{
    RT_UNUSED(pe);

    c5_pcnt_clear_unit(0);
    return RT_EOK;
}

/* ENABLE / DISABLE only. The framework handles CLEAR_COUNT and GET_TYPE itself
 * and calls ops->clear_count directly - it does not route those through here.
 * There is also no null check on ops->clear_count or ops->control in the
 * framework, so both have to be real functions; only ops->init is null-checked. */
static rt_err_t c5_pe_control(struct rt_pulse_encoder_device *pe,
                              rt_uint32_t cmd, void *args)
{
    RT_UNUSED(pe);
    RT_UNUSED(args);

    switch (cmd)
    {
    case PULSE_ENCODER_CMD_ENABLE:
        pcnt_ll_start_count(&PCNT, 0);
        return RT_EOK;

    case PULSE_ENCODER_CMD_DISABLE:
        pcnt_ll_stop_count(&PCNT, 0);
        return RT_EOK;

    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_pulse_encoder_ops c5_pe_ops =
{
    .init        = c5_pe_init,
    .get_count   = c5_pe_get_count,
    .clear_count = c5_pe_clear_count,
    .control     = c5_pe_control,
};

static rt_err_t c5_pe_register(void)
{
    s_pe_dev.type = SINGLE_PHASE_PULSE_ENCODER;
    s_pe_dev.ops  = &c5_pe_ops;

    return rt_device_pulse_encoder_register(&s_pe_dev, "pcnt0", RT_NULL);
}

rt_err_t rt_hw_pcnt_init(void)
{
    PERIPH_RCC_ATOMIC()
    {
        pcnt_ll_enable_bus_clock(0, true);
        pcnt_ll_reset_register(0);
    }

    pcnt_ll_set_clock_source(&PCNT, PCNT_CLK_SRC_APB);

    /* Register the device in the same band that enables the clock, so an
     * application calling rt_device_find("pcnt0") always succeeds. */
    return c5_pe_register();
}
INIT_BOARD_EXPORT(rt_hw_pcnt_init);

/*
 * Configure one unit to count positive edges on a pad.
 *
 * Channel 0 edge action: hold on the falling edge, increase on the rising edge -
 * that is "count rising edges". The control channel is left at its reset action
 * (keep), so no second pin is needed.
 */
rt_err_t rt_hw_pcnt_setup(int unit, rt_base_t pin)
{
    int sig = c5_pcnt_sig_idx(unit, 0);

    if (unit < 0 || unit >= C5_PCNT_UNITS || sig < 0)
    {
        return -RT_EINVAL;
    }

    if (pin < 0 || pin >= SOC_GPIO_PIN_COUNT ||
        ((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    /* Pad -> GPIO function, input on, no pulls. A pull would fight the driver
     * when the same pad is used as the software edge source. An output enable is
     * left alone: the caller may be driving this pad, which is exactly how the
     * self test works. */
    gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
    gpio_ll_input_enable(&GPIO, (uint8_t)pin);
    gpio_ll_pullup_dis(&GPIO, (uint8_t)pin);
    gpio_ll_pulldown_dis(&GPIO, (uint8_t)pin);

    esp_rom_gpio_connect_in_signal((uint32_t)pin, (uint32_t)sig, false);

    pcnt_ll_stop_count(&PCNT, (uint32_t)unit);
    pcnt_ll_clear_count(&PCNT, (uint32_t)unit);

    pcnt_ll_set_edge_action(&PCNT, (uint32_t)unit, 0,
                            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                            PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_ll_set_level_action(&PCNT, (uint32_t)unit, 0,
                             PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                             PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    /* Turn the glitch filter OFF.
     *
     * It is ON out of reset with a 16-cycle threshold (`ctrl.filter_en_un`
     * defaults to 1, `filter_thres_un` to 16 - pcnt_struct.h), and it is a real
     * filter: edges shorter than the threshold are discarded entirely. Software
     * driven edges from two consecutive stores are well inside that window, so
     * leaving the reset value gives a counter that reads 0 while every edge
     * provably appears on the pad.
     *
     * A real encoder wants this filter, with a threshold chosen for its edge
     * rate. A generic setter belongs in a driver API rather than hard-coded here;
     * what this one must not do is silently inherit a filter that eats the
     * signal. */
    pcnt_ll_enable_glitch_filter(&PCNT, (uint32_t)unit, false);

    /* No limits, no watch events: this is a plain counter. */
    pcnt_ll_disable_all_events(&PCNT, (uint32_t)unit);

    pcnt_ll_start_count(&PCNT, (uint32_t)unit);

    return RT_EOK;
}

int rt_hw_pcnt_get(int unit)
{
    return c5_pcnt_read_unit(unit);
}

void rt_hw_pcnt_clear(int unit)
{
    c5_pcnt_clear_unit(unit);
}

void rt_hw_pcnt_stop(int unit)
{
    if (unit >= 0 && unit < C5_PCNT_UNITS)
    {
        pcnt_ll_stop_count(&PCNT, (uint32_t)unit);
    }
}

/* ------------------------------------------------------------------ msh -- */

/*
 * `c5pcnt [gpio] [edges]`
 *
 * Drives the pad high/low `edges` times from software and compares the counter.
 * The expected count is exactly `edges` (rising edges), so this is a real
 * pass/fail rather than "something moved".
 */
int c5_pcnt_selftest(rt_base_t pin, rt_uint32_t edges)
{
    rt_uint32_t i;
    int counted;

    if (edges == 0u || edges > 1000u)
    {
        edges = 100u;
    }

    if (rt_hw_pcnt_setup(0, pin) != RT_EOK)
    {
        rt_kprintf("c5pcnt  : FAIL, unit 0 cannot use IO%d\n", (int)pin);
        return -1;
    }

    /* Drive the pad from the GPIO output register. This is what the counter is
     * measuring, so the output enable has to be under GPIO control, not the
     * peripheral's - see the note in twai.c/rmt.c about the same trap. */
    gpio_ll_output_enable(&GPIO, (uint8_t)pin);
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin, false, false);

    c5_pcnt_clear_unit(0);

    /* A short gap between edges.
     *
     * Two stores back to back produce a pulse a few tens of nanoseconds wide even
     * with the filter off, which is at the edge of what the input synchroniser
     * can see. 1 us is far below anything a real encoder would produce and far
     * above the pad's settling time. */
    for (i = 0; i < edges; i++)
    {
        gpio_ll_set_level(&GPIO, (uint8_t)pin, 1);
        esp_rom_delay_us(1);
        gpio_ll_set_level(&GPIO, (uint8_t)pin, 0);
        esp_rom_delay_us(1);
    }

    counted = c5_pcnt_read_unit(0);

    rt_kprintf("c5pcnt  : unit 0 on IO%d, drove %u rising edge(s), counter reads %d\n",
               (int)pin, (unsigned)edges, counted);

    if (counted != (int)edges)
    {
        rt_kprintf("c5pcnt  : FAIL, expected %u and counted %d\n",
                   (unsigned)edges, counted);
        return -1;
    }

    /* Also exercise the RT-Thread device path, so the registration is not just
     * declared - the count has to come back through rt_device_read(). */
    {
        rt_device_t dev = rt_device_find("pcnt0");
        rt_int32_t  via_dev = 0;

        if (dev == RT_NULL)
        {
            rt_kprintf("c5pcnt  : FAIL, pcnt0 is not registered\n");
            return -1;
        }

        if (rt_device_open(dev, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
        {
            rt_kprintf("c5pcnt  : FAIL, pcnt0 did not open\n");
            return -1;
        }

        if (rt_device_read(dev, 0, &via_dev, 1) != 1)
        {
            rt_device_close(dev);
            rt_kprintf("c5pcnt  : FAIL, rt_device_read did not return a sample\n");
            return -1;
        }

        rt_device_close(dev);

        if (via_dev != counted)
        {
            rt_kprintf("c5pcnt  : FAIL, rt_device_read returned %d but the unit "
                       "holds %d\n", (int)via_dev, counted);
            return -1;
        }

        rt_kprintf("c5pcnt  : pcnt0 device path agrees (%d)\n", (int)via_dev);
    }

    rt_kprintf("c5pcnt  : PASS\n");
    return 0;
}

#endif /* SOC_PCNT_SUPPORTED */



