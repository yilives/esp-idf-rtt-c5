/*
 * ESP32-S31 SAR ADC (oneshot) as an rt_adc_device.
 *
 * Two units, 8 channels each, 17-bit raw results. Registered as `adc1` and
 * `adc2`, so `rt_adc_enable()` / `rt_adc_read()` work and any RT-Thread sensor
 * package built on the ADC framework can use them.
 *
 * ---------------------------------------------------------------------------
 * What is deliberately NOT claimed here:
 *
 *   - NO voltage conversion. This port has no calibration path: the S31 has no
 *     SOC_ADC_CALIBRATION_V1_SUPPORTED and no RTC controller, so the IDF
 *     calibration component cannot produce raw-to-mV coefficients for it. The
 *     driver therefore reports the 17-bit raw code and get_vref() returns -1
 *     ("unknown") rather than inventing a reference voltage. A number that was
 *     not measured is worse than no number - the ADC would silently report
 *     wrong millivolts and every downstream calculation would inherit the lie.
 *
 *   - NO continuous/DMA mode. Oneshot is the framework's model and the only
 *     mode that can be verified without a signal generator.
 *
 * ---------------------------------------------------------------------------
 * The unit/channel-to-pin map is fixed silicon, not a board choice
 * (soc/adc_channel.h):
 *
 *   ADC1 ch0..7 -> GPIO42..49
 *   ADC2 ch0..7 -> GPIO50..57
 *
 * Note the collision with the rest of this board: **GPIO50/51 are the I2C
 * pins**. ADC2 channel 0 and 1 sit on the same pads, so enabling ADC2 while
 * i2c0 is up means both peripherals drive one pad - the analog function and the
 * digital one cannot be used at the same time. `s31adc` refuses to read ADC2
 * unless the caller passes `force`, and the log says why.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/adc_channel.h"
#include "hal/adc_ll.h"
#include "hal/gpio_ll.h"
#include "hal/adc_types.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"

#if defined(SOC_ADC_SUPPORTED)

#include "port_adc.h"

/* Per-unit state: which channel is currently selected. The hardware holds one
 * pattern entry, so a read has to reprogram it every time. */
struct s31_adc
{
    struct rt_adc_device parent;
    adc_unit_t           unit;
    rt_int8_t            channel;     /* last selected, -1 = none */
    rt_bool_t            powered;
};

static struct s31_adc s_adc[2];

/* ADC1 ch N -> GPIO(42+N); ADC2 ch N -> GPIO(50+N). Kept as two expressions
 * rather than a table because that is exactly what soc/adc_channel.h encodes. */
static rt_base_t s31_adc_pin(adc_unit_t unit, rt_int8_t ch)
{
    if (ch < 0 || ch >= SOC_ADC_MAX_CHANNEL_NUM)
    {
        return -1;
    }

    return (unit == ADC_UNIT_1) ? (42 + ch) : (50 + ch);
}

/*
 * Park the pad for analog use.
 *
 * This is the step that is easy to skip and impossible to notice: a pad left in
 * its digital default still has the input buffer and the pull resistors
 * connected, so the SAR measures the pad's own driver rather than the voltage
 * on it. IDF's gpio_config_as_analog() does exactly these five things and no
 * more - the pad is left floating, with PIN_FUNC_GPIO so the matrix is out of
 * the way.
 *
 * Symptom when omitted: every channel reads 0 (the digital input buffer holds
 * the pad low), which looks like "the ADC is broken" rather than "the pin is
 * still a GPIO".
 */
static void s31_adc_park_pad(rt_base_t pin)
{
    if (pin < 0 || ((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) == 0)
    {
        return;
    }

    gpio_ll_input_disable(&GPIO, (uint8_t)pin);
    gpio_ll_output_disable(&GPIO, (uint8_t)pin);
    gpio_ll_pullup_dis(&GPIO, (uint8_t)pin);
    gpio_ll_pulldown_dis(&GPIO, (uint8_t)pin);
    gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
}

static rt_err_t s31_adc_enabled(struct rt_adc_device *device, rt_int8_t channel,
                                rt_bool_t enabled)
{
    struct s31_adc *adc = (struct s31_adc *)device;

    if (channel < 0 || channel >= SOC_ADC_MAX_CHANNEL_NUM)
    {
        return -RT_EINVAL;
    }

    if (enabled)
    {
        /* The pad has to be parked for analog before the SAR can see it. */
        s31_adc_park_pad(s31_adc_pin(adc->unit, channel));

        /* Program the pattern register HERE, not in convert().
         *
         * The first version set adc->channel here as a bookkeeping value and
         * left the hardware alone, on the theory that convert() would program
         * it on a channel change. Since this line already recorded the channel,
         * the `if (adc->channel != channel)` in convert() was false for every
         * single read - the pattern table kept its reset value 0x00f00000
         * (channel 15, an invalid entry) and every conversion returned 0.
         * `s31adcdbg` showed the conversion itself completing with raw=1807, so
         * the SAR was fine and only the channel selection was missing.
         *
         * Doing it here also matches the framework: rt_adc_enable(dev, ch) is
         * "prepare this channel", rt_adc_read(dev, ch) is "give me a sample". */
        adc_oneshot_ll_set_channel(adc->unit, (adc_channel_t)channel);

        /* Power the unit up. adc_ll_set_power_manage() also enables the
         * reference generator, which the SAR needs before any conversion - a
         * unit that is triggered without it returns zeros. */
        adc_ll_set_power_manage(adc->unit, ADC_LL_POWER_SW_ON);
        adc->powered = RT_TRUE;
        adc->channel = channel;
    }
    else
    {
        adc_oneshot_ll_disable_channel(adc->unit);
        adc_ll_set_power_manage(adc->unit, ADC_LL_POWER_SW_OFF);
        adc->powered = RT_FALSE;
        adc->channel = -1;
    }

    return RT_EOK;
}

static rt_err_t s31_adc_convert(struct rt_adc_device *device, rt_int8_t channel,
                                rt_uint32_t *value)
{
    struct s31_adc *adc = (struct s31_adc *)device;
    rt_uint32_t event = (adc->unit == ADC_UNIT_1) ? ADC_LL_EVENT_ADC1_ONESHOT_DONE
                                               : ADC_LL_EVENT_ADC2_ONESHOT_DONE;
    rt_uint32_t guard;

    if (value == RT_NULL || channel < 0 || channel >= SOC_ADC_MAX_CHANNEL_NUM)
    {
        return -RT_EINVAL;
    }

    if (!adc->powered)
    {
        return -RT_ERROR;
    }

    /*
     * Reprogram the pattern register on EVERY conversion.
     *
     * Not just when the channel changes, and this is the second version of this
     * bug: the table holds one entry and each conversion advances the pattern
     * pointer past it, so the next conversion finds no entry and returns 0.
     * Measured: first read 1771, second and third 0.
     *
     * adc_oneshot_ll_program_pattern() resets the pointer (sar1_patt_p_clear
     * pulse) as part of programming, so calling it every time is both
     * necessary and sufficient. IDF does the same: adc_oneshot_hal_setup() runs
     * before each convert, not once per channel change.
     */
    adc_oneshot_ll_set_channel(adc->unit, (adc_channel_t)channel);
    adc->channel = channel;

    /* A stale done bit from a previous conversion makes the poll below return
     * immediately and read the old result. */
    adc_oneshot_ll_clear_event(event);
    adc_oneshot_ll_enable(adc->unit);
    adc_oneshot_ll_start(adc->unit);

    /* Bounded poll. A single conversion is a few microseconds; the guard only
     * exists so a mis-clocked unit reports instead of hanging the shell. */
    guard = 0;
    while (!adc_oneshot_ll_get_event(event))
    {
        if (++guard > 2000000u)
        {
            adc_oneshot_ll_disable_all_unit();
            return -RT_ETIMEOUT;
        }
    }

    /* Two DT-proven delays, both from IDF's adc_hal_onetime_start():
     *
     *   1. After DONE the SAR needs a moment to synchronise the result into the
     *      data register; reading immediately can return the PREVIOUS value or
     *      zero. ADC_LL_DELAY_CYCLE_AFTER_DONE_SIGNAL is 0 on this part, so IDF
     *      uses a fixed read_delay_us derived from the clock - this driver just
     *      waits one SAR clock period, which is the conservative form.
     *
     *   2. The trigger_start bit is write-through/self-clearing. Without a pause
     *      between asserting it and the poll, the first poll can observe the
     *      previous conversion's done bit before the new one has even been
     *      latched.
     *
     * Skipping these is exactly how a working ADC reports a constant 0. */
    esp_rom_delay_us(20);

    *value = adc_oneshot_ll_get_raw_result(adc->unit);

    adc_oneshot_ll_disable_all_unit();

    return RT_EOK;
}

static rt_uint8_t s31_adc_resolution(struct rt_adc_device *device)
{
    RT_UNUSED(device);

    return SOC_ADC_RTC_MAX_BITWIDTH;
}

static rt_int16_t s31_adc_vref(struct rt_adc_device *device)
{
    RT_UNUSED(device);

    /* Unknown, and said so. See the file header: this port has no calibration
     * path on this SoC, so there is no reference voltage to report. Returning a
     * "typical" 1100 here would look authoritative and be wrong by several
     * percent on any given board.
     *
     * 0, not -1. rt_adc_voltage() reads this as
     *
     *     vref = dev->ops->get_vref(dev);
     *     if (vref == 0) goto _voltage_exit;      -> returns 0
     *
     * so 0 is the framework's "cannot compute" value, and it produces a clean 0 mV.
     * -1 slips past that check and is multiplied into the result:
     *
     *     voltage = value * -1 / ((1 << resolution) - 1)
     *
     * which `adc voltage` then prints with the sign in the middle of the digits -
     * measured as "-32.-768V". Five upstream BSP drv_adc.c files return 0 for this
     * case and none return -1. */
    return 0;
}

static const struct rt_adc_ops s31_adc_ops =
{
    .enabled        = s31_adc_enabled,
    .convert        = s31_adc_convert,
    .get_resolution = s31_adc_resolution,
    .get_vref       = s31_adc_vref,
};

/* Shared bring-up: clocks, reset, divider, controller. Done once, not per
 * unit - the two SARs share one register block and one clock tree. */
static rt_err_t s31_adc_hw_init(void)
{
    /* LP_PERI_CLKRST gates the ADC block; the reset is separate. Both are
     * macros that require an RCC atomic environment in scope. */
    PERIPH_RCC_ATOMIC()
    {
        adc_ll_enable_bus_clock(true);
        adc_ll_reset_register();
        adc_ll_enable_func_clock(true);
    }

    /* XTAL, not RC_FAST: the conversion result is only as stable as the clock
     * that times the SAR, and RC_FAST is the one clock on this chip that is
     * calibrated by measuring it. */
    adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_XTAL);
    adc_ll_digi_controller_clk_div(ADC_LL_CLKM_DIV_NUM_DEFAULT,
                                   ADC_LL_CLKM_DIV_B_DEFAULT,
                                   ADC_LL_CLKM_DIV_A_DEFAULT);

    adc_ll_enable_refgen();

    return RT_EOK;
}

static rt_err_t s31_adc_register(adc_unit_t unit, const char *name)
{
    struct s31_adc *adc = &s_adc[(unit == ADC_UNIT_1) ? 0 : 1];

    adc->unit    = unit;
    adc->channel = -1;
    adc->powered = RT_FALSE;

    return rt_hw_adc_register(&adc->parent, name, &s31_adc_ops, RT_NULL);
}

/* ------------------------------------------------------------------ msh -- */

/*
 * `s31adc [unit] [channel]` - read one channel and print the raw code.
 *
 * ADC2 is refused unless `force` is the third word, because its channels 0/1
 * are the I2C pads (GPIO50/51): reading them while i2c0 owns the pads measures
 * a driven digital line, not an analog input, and the number would be
 * meaningless even though it looks plausible.
 */
static int cmd_adc(int argc, char **argv)
{
    rt_uint32_t unit = 1;
    rt_uint32_t ch = 0;
    rt_bool_t force = RT_FALSE;
    rt_adc_device_t dev;
    rt_uint32_t value = 0;
    rt_err_t rc;
    char name[8];
    int i;

    if (argc >= 2)
    {
        unit = (rt_uint32_t)atoi(argv[1]);
    }
    if (argc >= 3)
    {
        ch = (rt_uint32_t)atoi(argv[2]);
    }
    if (argc >= 4 && rt_strcmp(argv[3], "force") == 0)
    {
        force = RT_TRUE;
    }

    if (argc < 2 || unit < 1 || unit > 2 || ch >= SOC_ADC_MAX_CHANNEL_NUM)
    {
        rt_kprintf("usage: s31adc <1|2> <0..%d> [force]\n",
                   SOC_ADC_MAX_CHANNEL_NUM - 1);
        rt_kprintf("       adc1 ch0..7 = GPIO42..49\n");
        rt_kprintf("       adc2 ch0..7 = GPIO50..57\n");
        rt_kprintf("       adc2 ch0/1 are the I2C pads; pass 'force' to read anyway\n");
        return -1;
    }

    if (unit == 2 && ch <= 1 && !force)
    {
        rt_kprintf("s31adc: adc2 ch%u is GPIO%u, the i2c0 pad.\n",
                   (unsigned)ch, (unsigned)(50 + ch));
        rt_kprintf("  Reading it while i2c0 is up measures a driven digital\n");
        rt_kprintf("  line, not an analog input. Use 'force' if i2c0 is idle.\n");
        return -1;
    }

    name[0] = 'a'; name[1] = 'd'; name[2] = 'c';
    name[3] = (char)('0' + unit); name[4] = '\0';

    dev = (rt_adc_device_t)rt_device_find(name);
    if (dev == RT_NULL)
    {
        rt_kprintf("s31adc: %s not registered\n", name);
        return -1;
    }

    rc = rt_adc_enable(dev, (rt_int8_t)ch);
    if (rc != RT_EOK)
    {
        rt_kprintf("s31adc: enable failed %d\n", rc);
        return -1;
    }

    /* Three reads: the first conversion after power-up is the one that can be
     * short by a couple of LSB while the reference settles. Reporting all three
     * makes that visible instead of hiding it behind an average. */
    rt_kprintf("s31adc: %s ch%u (GPIO%u), %u-bit raw\n",
               name, (unsigned)ch, (unsigned)s31_adc_pin((adc_unit_t)(unit - 1), (rt_int8_t)ch),
               (unsigned)SOC_ADC_RTC_MAX_BITWIDTH);

    for (i = 0; i < 3; i++)
    {
        rc = rt_adc_read(dev, (rt_int8_t)ch);
        if ((rt_int32_t)rc < 0)
        {
            rt_kprintf("  read failed %d\n", (int)rc);
            rt_adc_disable(dev, (rt_int8_t)ch);
            return -1;
        }
        value = (rt_uint32_t)rc;
        rt_kprintf("  read %d: %u\n", i + 1, (unsigned)value);
    }

    rt_adc_disable(dev, (rt_int8_t)ch);

    rt_kprintf("s31adc: no calibration on this port - the raw code is the result,\n");
    rt_kprintf("  not millivolts. See the note in port_adc.h.\n");

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_adc, s31adc, read a SAR ADC channel (raw 17-bit code));

/*
 * `s31adcdbg` - dump the ADC register block around one conversion.
 *
 * Added because `s31adc` returns a constant 0 and there are three very different
 * reasons that can happen: the conversion never starts, it starts but the result
 * register I read is not the one S31 populates, or the pad really is at 0 V.
 * Guessing between them is what turned the PSRAM bug into a four-round detour,
 * so this dumps the state instead.
 */
static int cmd_adcdbg(int argc, char **argv)
{
    rt_uint32_t unit = 1;
    rt_uint32_t event;

    if (argc >= 2)
    {
        unit = (rt_uint32_t)atoi(argv[1]);
    }

    event = (unit == 1) ? ADC_LL_EVENT_ADC1_ONESHOT_DONE : ADC_LL_EVENT_ADC2_ONESHOT_DONE;

    rt_kprintf("adc%d before: int_raw=0x%08x ctrl0=0x%08x ctrl1=0x%08x\n",
               (int)unit, (unsigned)ADC.int_raw.val,
               (unsigned)ADC.ctrl0.val, (unsigned)ADC.ctrl1.val);
    rt_kprintf("  ctrl2=0x%08x ctrl_date=0x%08x ref=0x%08x\n",
               (unsigned)ADC.ctrl2.val, (unsigned)ADC.ctrl_date.val,
               (unsigned)ADC.ref_control.val);
    rt_kprintf("  sar1_data_status=0x%08x sar2=0x%08x\n",
               (unsigned)ADC.sar1_data_status.val,
               (unsigned)ADC.sar2_data_status.val);
    rt_kprintf("  patt_tab1 sar1=0x%08x sar2=0x%08x\n",
               (unsigned)ADC.sar1_patt_tab1.val,
               (unsigned)ADC.sar2_patt_tab1.val);

    {
        adc_unit_t u = (unit == 1) ? ADC_UNIT_1 : ADC_UNIT_2;
        rt_uint32_t guard = 0;
        rt_bool_t got;

        adc_ll_set_power_manage(u, ADC_LL_POWER_SW_ON);
        adc_oneshot_ll_set_channel(u, ADC_CHANNEL_0);
        adc_oneshot_ll_clear_event(event);
        adc_oneshot_ll_enable(u);
        adc_oneshot_ll_start(u);

        while (!adc_oneshot_ll_get_event(event))
        {
            if (++guard > 2000000u)
            {
                break;
            }
        }

        got = adc_oneshot_ll_get_event(event);

        rt_kprintf("adc%d conv: done=%d loops=%u\n",
                   (int)unit, (int)got, (unsigned)guard);
        rt_kprintf("  int_raw=0x%08x int_clr=0x%08x\n",
                   (unsigned)ADC.int_raw.val, (unsigned)ADC.int_clr.val);
        rt_kprintf("  sar1_data_status=0x%08x sar2=0x%08x\n",
                   (unsigned)ADC.sar1_data_status.val,
                   (unsigned)ADC.sar2_data_status.val);
        rt_kprintf("  raw(unit)=%u\n",
                   (unsigned)adc_oneshot_ll_get_raw_result(u));
        rt_kprintf("  ctrl0=0x%08x ctrl1=0x%08x\n",
                   (unsigned)ADC.ctrl0.val, (unsigned)ADC.ctrl1.val);

        adc_oneshot_ll_disable_all_unit();
        adc_ll_set_power_manage(u, ADC_LL_POWER_SW_OFF);
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_adcdbg, s31adcdbg, dump ADC registers around one conversion);

rt_err_t rt_hw_adc_init(void)
{
    rt_err_t rc = s31_adc_hw_init();

    if (rc != RT_EOK)
    {
        rt_kprintf("adc: hardware init failed %d\n", rc);
        return rc;
    }

    rc = s31_adc_register(ADC_UNIT_1, "adc1");
    if (rc != RT_EOK)
    {
        rt_kprintf("adc1: register failed %d\n", rc);
        return rc;
    }

    rc = s31_adc_register(ADC_UNIT_2, "adc2");
    if (rc != RT_EOK)
    {
        rt_kprintf("adc2: register failed %d\n", rc);
    }

    return rc;
}
INIT_BOARD_EXPORT(rt_hw_adc_init);

#endif /* SOC_ADC_SUPPORTED */
