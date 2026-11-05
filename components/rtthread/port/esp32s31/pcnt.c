/*
 * ESP32-S31 PCNT (pulse counter), registered as an RT-Thread pulse-encoder device.
 *
 * This used to say "RT-Thread has no pulse-counter class" and expose only a private
 * C API. That was wrong: 5.4.0 ships components/drivers/misc/pulse_encoder.c and
 * include/drivers/pulse_encoder.h behind RT_USING_PULSE_ENCODER, and the framework
 * even carries a `#ifndef RT_USING_DEVICE_OPS` branch - which is this port's config.
 * Registering "pcnt0" means the count is reachable the RT-Thread way:
 *
 *     rt_device_t d = rt_device_find("pcnt0");
 *     rt_int32_t count;
 *     rt_device_open(d, RT_DEVICE_OFLAG_RDONLY);
 *     rt_device_read(d, 0, &count, 1);    -- writes 4 bytes regardless of the size arg
 *     rt_device_control(d, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
 *
 * `rt_hw_pcnt_*` stay as thin wrappers over the device, because mcpwm.c and rmt.c use
 * the counter as a measurement instrument rather than as a user-facing peripheral.
 *
 * Four units per group, two groups on this SoC; `s31pcnt` exposes group 0.
 * Only unit 0 is registered: four devices would imply four verified counters.
 *
 * ---------------------------------------------------------------------------
 * How this is verified without any external wiring.
 *
 * A PCNT unit counts edges seen on a pad, and the pad's input path sees whatever
 * the pad is driven to - including by its own output driver. So the test drives
 * a GPIO high and low from software N times and lets PCNT count the edges on
 * that same pin. That is a genuine round trip through the input matrix, the
 * glitch filter and the counter; it just does not need a jumper.
 *
 * The alternative (wire MCPWM to PCNT, or use a signal generator) is what a
 * bench test would do, and it cannot run here. This still exercises everything
 * except an external driver's edge shape.
 *
 * ---------------------------------------------------------------------------
 * S31 specifics worth remembering:
 *
 *  - Two groups, PCNT0 and PCNT1, each with 4 units and 2 channels per unit.
 *    The group's clock gate and reset live in HP_SYS_CLKRST (pcnt_ctrl0) and the
 *    APB clock is the only legal source (`pcnt_ll_set_clock_source` asserts
 *    anything else).
 *
 *  - The count register is 16-bit *signed* and the read helper sign-extends it.
 *    A count that "wraps" at 32767 is not a bug in this driver.
 *
 *  - Filter thresholds are in APB clock cycles, and the filter is off by
 *    default. Software-driven edges are slow enough not to need it, but a real
 *    encoder usually does.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_ll.h"
#include "hal/pcnt_ll.h"
#include "hal/pcnt_types.h"
#include "esp_rom_gpio.h"
#include "esp_attr.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_sys.h"

#if defined(SOC_PCNT_SUPPORTED)

#include "port_pcnt.h"

#define S31_PCNT_GROUP      0
#define S31_PCNT_UNITS      4

static pcnt_dev_t *s31_pcnt_hw(int group)
{
    return (group == 0) ? &PCNT0 : &PCNT1;
}

/* Signal indices for group 0 unit N, channel C. The table is laid out as
 * PCNT0_SIG_CH<c>_IN<unit>, i.e. channel-major, which is the opposite of how
 * every other peripheral here is indexed - worth spelling out. */
static int s31_pcnt_sig_idx(int unit, int channel)
{
    static const int idx[2][S31_PCNT_UNITS] =
    {
        { PCNT0_SIG_CH0_IN0_IDX, PCNT0_SIG_CH0_IN1_IDX,
          PCNT0_SIG_CH0_IN2_IDX, PCNT0_SIG_CH0_IN3_IDX },
        { PCNT0_SIG_CH1_IN0_IDX, PCNT0_SIG_CH1_IN1_IDX,
          PCNT0_SIG_CH1_IN2_IDX, PCNT0_SIG_CH1_IN3_IDX },
    };

    if (unit < 0 || unit >= S31_PCNT_UNITS || channel < 0 || channel > 1)
    {
        return -1;
    }

    return idx[channel][unit];
}

/* The RT-Thread device. One unit is registered - see the header for why. */
static struct rt_pulse_encoder_device s31_pe_dev;

/* One implementation of "read the counter" and one of "clear it", shared by the
 * device ops below and the public API further down. */
static int s31_pcnt_read_unit(int unit)
{
    if (unit < 0 || unit >= S31_PCNT_UNITS)
    {
        return 0;
    }

    return pcnt_ll_get_count(s31_pcnt_hw(S31_PCNT_GROUP), unit);
}

static void s31_pcnt_clear_unit(int unit)
{
    if (unit >= 0 && unit < S31_PCNT_UNITS)
    {
        pcnt_ll_clear_count(s31_pcnt_hw(S31_PCNT_GROUP), unit);
    }
}

static rt_err_t s31_pe_init(struct rt_pulse_encoder_device *pe)
{
    RT_UNUSED(pe);

    /* Nothing to do: rt_hw_pcnt_init() runs from the board init band and has
     * already enabled the group clock and selected the APB source. */
    return RT_EOK;
}

static rt_int32_t s31_pe_get_count(struct rt_pulse_encoder_device *pe)
{
    RT_UNUSED(pe);

    return (rt_int32_t)s31_pcnt_read_unit(0);
}

static rt_err_t s31_pe_clear_count(struct rt_pulse_encoder_device *pe)
{
    RT_UNUSED(pe);

    s31_pcnt_clear_unit(0);
    return RT_EOK;
}

/*
 * ENABLE / DISABLE only. The framework handles CLEAR_COUNT and GET_TYPE itself and
 * calls ops->clear_count directly - it does not route those through here.
 *
 * There is also no null check on ops->clear_count or ops->control in the framework, so
 * both have to be real functions; only ops->init is null-checked.
 */
static rt_err_t s31_pe_control(struct rt_pulse_encoder_device *pe,
                               rt_uint32_t cmd, void *args)
{
    RT_UNUSED(pe);
    RT_UNUSED(args);

    switch (cmd)
    {
    case PULSE_ENCODER_CMD_ENABLE:
        pcnt_ll_start_count(s31_pcnt_hw(S31_PCNT_GROUP), 0);
        return RT_EOK;

    case PULSE_ENCODER_CMD_DISABLE:
        pcnt_ll_stop_count(s31_pcnt_hw(S31_PCNT_GROUP), 0);
        return RT_EOK;

    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_pulse_encoder_ops s31_pe_ops =
{
    .init        = s31_pe_init,
    .get_count   = s31_pe_get_count,
    .clear_count = s31_pe_clear_count,
    .control     = s31_pe_control,
};

static rt_err_t s31_pe_register(void)
{
    s31_pe_dev.type = SINGLE_PHASE_PULSE_ENCODER;
    s31_pe_dev.ops  = &s31_pe_ops;

    return rt_device_pulse_encoder_register(&s31_pe_dev, "pcnt0", RT_NULL);
}

rt_err_t rt_hw_pcnt_init(void)
{
    PERIPH_RCC_ATOMIC()
    {
        pcnt_ll_enable_bus_clock(S31_PCNT_GROUP, true);
        pcnt_ll_reset_register(S31_PCNT_GROUP);
    }

    pcnt_ll_set_clock_source(s31_pcnt_hw(S31_PCNT_GROUP), PCNT_CLK_SRC_APB);

    /* Register the device in the same band that enables the clock, so an application
     * calling rt_device_find("pcnt0") always succeeds. */
    return s31_pe_register();
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
    pcnt_dev_t *hw = s31_pcnt_hw(S31_PCNT_GROUP);
    int sig = s31_pcnt_sig_idx(unit, 0);

    if (unit < 0 || unit >= S31_PCNT_UNITS || sig < 0)
    {
        return -RT_EINVAL;
    }

    if (((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    /* Pad -> GPIO function, input on, no pulls. A pull would fight the driver
     * when the same pad is used as the software edge source. */
    gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
    gpio_ll_input_enable(&GPIO, (uint8_t)pin);
    gpio_ll_pullup_dis(&GPIO, (uint8_t)pin);
    gpio_ll_pulldown_dis(&GPIO, (uint8_t)pin);

    esp_rom_gpio_connect_in_signal((uint32_t)pin, sig, false);

    pcnt_ll_stop_count(hw, unit);
    pcnt_ll_clear_count(hw, unit);

    pcnt_ll_set_edge_action(hw, unit, 0,
                            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                            PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_ll_set_level_action(hw, unit, 0,
                             PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                             PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    /* No limits, no watch events: this is a plain counter. */
    pcnt_ll_disable_all_events(hw, unit);

    pcnt_ll_start_count(hw, unit);

    return RT_EOK;
}

int rt_hw_pcnt_get(int unit)
{
    return s31_pcnt_read_unit(unit);
}

void rt_hw_pcnt_clear(int unit)
{
    s31_pcnt_clear_unit(unit);
}

void rt_hw_pcnt_stop(int unit)
{
    if (unit >= 0 && unit < S31_PCNT_UNITS)
    {
        pcnt_ll_stop_count(s31_pcnt_hw(S31_PCNT_GROUP), (uint32_t)unit);
    }
}

/* ------------------------------------------------------------------ msh -- */

/*
 * `s31pcnt <unit> <gpio> [edges]`
 *
 * Drives the pad high/low `edges` times from software and compares the counter.
 * The expected count is exactly `edges` (rising edges), so this is a real
 * pass/fail rather than "something moved".
 */
static int cmd_pcnt(int argc, char **argv)
{
    rt_uint32_t unit = 0;
    rt_base_t pin = 16;
    rt_uint32_t edges = 100;
    rt_uint32_t i;
    int counted;
    rt_base_t level;

    if (argc >= 2)
    {
        unit = (rt_uint32_t)atoi(argv[1]);
    }
    if (argc >= 3)
    {
        pin = (rt_base_t)atoi(argv[2]);
    }
    if (argc >= 4)
    {
        edges = (rt_uint32_t)atoi(argv[3]);
    }

    if (argc < 3 || unit >= S31_PCNT_UNITS || edges == 0)
    {
        rt_kprintf("usage: s31pcnt <unit 0..%d> <gpio> [edges]\n",
                   S31_PCNT_UNITS - 1);
        return -1;
    }

    if (rt_hw_pcnt_setup((int)unit, pin) != RT_EOK)
    {
        rt_kprintf("s31pcnt: setup failed (unit %u, gpio %d)\n",
                   (unsigned)unit, (int)pin);
        return -1;
    }

    /* The same pad is now both the counter's input and a software-driven
     * output. That is the whole trick - no jumper needed.
     *
     * Drive it through the pin device, not with a single LL call. Enabling the
     * output driver is only one of three things an output pad needs: the output
     * multiplexer must also point at the GPIO latch and the output-enable must be
     * handed to the GPIO block. `gpio_ll_output_enable()` alone leaves whatever a
     * previous owner programmed in place, so the latch write may never reach the
     * pad.
     *
     * Measured before this fix: `s31pcnt 0 <pin> 20` passed on pins 4-7, 16-20, 30,
     * 40 and 60, and counted 0 on 8-15 and 50 - same code, same 2 us pulse. The
     * difference was the pads' output-multiplexer reset state, not the counter and
     * not the edge width. */
    rt_pin_mode(pin, PIN_MODE_OUTPUT);

    rt_kprintf("s31pcnt: unit %u on GPIO%d, driving %u rising edges\n",
               (unsigned)unit, (int)pin, (unsigned)edges);

    /* Interrupts off for the burst: a tick landing between the edge and the
     * read is harmless, but keeping the burst atomic makes the expected count
     * exact instead of "approximately edges". */
    level = rt_hw_interrupt_disable();

    gpio_ll_set_level(&GPIO, (uint8_t)pin, 0);
    for (i = 0; i < edges; i++)
    {
        gpio_ll_set_level(&GPIO, (uint8_t)pin, 1);
        /* The pulse has to be wide enough for the counter to see it.
         *
         * PCNT samples the pad on the APB clock (80 MHz, 12.5 ns). Writing the
         * level register twice back to back produces a high period of a few
         * CPU instructions - about 10 ns at 320 MHz - which the sampler can
         * miss entirely. Measured: 100 driven edges counted as 1.
         *
         * Two microseconds is ~160 APB cycles per level: four orders of
         * magnitude of margin, and 100 edges still finish in 200 us. */
        esp_rom_delay_us(2);
        gpio_ll_set_level(&GPIO, (uint8_t)pin, 0);
        esp_rom_delay_us(2);
    }

    rt_hw_interrupt_enable(level);

    counted = rt_hw_pcnt_get((int)unit);

    /* Stop counting and release the pad so the next command starts clean. */
    pcnt_ll_stop_count(s31_pcnt_hw(S31_PCNT_GROUP), (uint32_t)unit);
    gpio_ll_output_disable(&GPIO, (uint8_t)pin);

    rt_kprintf("  expected %u, counted %d\n", (unsigned)edges, counted);

    if (counted == (int)edges)
    {
        rt_kprintf("s31pcnt: PASS\n");
        return 0;
    }

    rt_kprintf("s31pcnt: FAIL (delta %d)\n", counted - (int)edges);
    if (counted == 0)
    {
        /* Zero is a different failure from a short count: nothing reached the
         * counter at all, which points at the pad's output path rather than at
         * the edge rate. Saying "retry with fewer edges" here sent the reader in
         * the wrong direction for a whole round. */
        rt_kprintf("  Zero means no edge reached the counter: check the pad's\n");
        rt_kprintf("  output path (mux source / output enable), not the edge rate.\n");
    }
    else
    {
        rt_kprintf("  A short count usually means the edges were faster than the\n");
        rt_kprintf("  counter can resolve; retry with fewer edges.\n");
    }
    return -1;
}
MSH_CMD_EXPORT_ALIAS(cmd_pcnt, s31pcnt, count software-driven edges on a pin);

#endif /* SOC_PCNT_SUPPORTED */
