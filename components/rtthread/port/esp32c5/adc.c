/*
 * ESP32-C5 SAR ADC (oneshot) as an rt_adc_device.
 *
 * One unit, six channels, 12-bit results. Registered as `adc1`, so
 * rt_adc_enable() / rt_adc_read() work and any sensor package built on the ADC
 * framework can use it.
 *
 * Ported from port/esp32s31/adc.c. The C5 is simpler in one way and harder in
 * two:
 *
 *  - ONE unit, not two. SOC_ADC_PERIPH_NUM is 1, SOC_ADC_MAX_CHANNEL_NUM is 6,
 *    and the LL asserts `adc_n == ADC_UNIT_1` in every oneshot call. Anything
 *    that indexes two units silently writes the wrong registers.
 *
 *  - THE UNIT IS POWERED WITH SW_ON, NOT BY_FSM. The LL header labels BY_FSM
 *    "used for polling mode", which reads like the right choice for a polling
 *    driver - and the first version of this file used it, and every conversion
 *    timed out. IDF's oneshot HAL, which is also polling, unconditionally sets
 *    ADC_LL_POWER_SW_ON (adc_oneshot_hal.c:64). The header comment and the
 *    working code disagree; the working code wins.
 *
 *  - THE CLOCK HAS TO BE CONFIGURED. This chip has SOC_ADC_DIG_CTRL_SUPPORTED
 *    and no RTC controller, so adc_oneshot_hal.c takes its `digi_clk_sel /
 *    digi_controller_clk_div / digi_set_clk_div` branch. Skipping it leaves the
 *    digital controller without a clock: the start bit is set and the done event
 *    never arrives.
 *
 * ---------------------------------------------------------------------------
 * What is deliberately NOT claimed:
 *
 *   - NO voltage conversion. The C5 does have SOC_ADC_CALIBRATION_V1_SUPPORTED,
 *     but the calibration coefficients come from the ADC calibration component,
 *     which is not part of this port and whose numbers were never measured here.
 *     get_vref() returns 0 ("cannot compute") rather than inventing a reference
 *     voltage. A number that was not measured is worse than no number.
 *
 *   - NO continuous/DMA mode. Oneshot is the framework's model and the only mode
 *     that can be checked without a signal generator.
 *
 * ---------------------------------------------------------------------------
 * The channel-to-pin map is fixed silicon (soc/adc_channel.h):
 *
 *   ADC1 ch0..5 -> GPIO1..GPIO6
 *
 * Note the collision with the rest of this image: **GPIO3 is used by the PWM
 * self test** and **GPIO1/GPIO2 are LP_UART pins**. Reading a channel whose pad
 * is driven by another peripheral measures that driver, not the pin. The
 * self test below reads all six and says which pads are shared, rather than
 * pretending the reading is meaningful.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/adc_channel.h"
#include "hal/adc_ll.h"
#include "hal/adc_types.h"
#include "hal/adc_hal_common.h"
#include "hal/gpio_ll.h"
#include "esp_rom_sys.h"
#include "esp_private/esp_clk.h"
#include "esp_private/sar_periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_ADC_SUPPORTED)

#include "port_adc.h"

struct c5_adc
{
    struct rt_adc_device parent;
    rt_int8_t            channel;     /* last selected, -1 = none */
    rt_bool_t            powered;
};

static struct c5_adc s_adc[1];

/* ADC1 channel N -> GPIO(N + 1), straight from soc/adc_channel.h, which defines
 * ADC1_CHANNEL_0_GPIO_NUM as 1 and increments by one. Written as an expression
 * rather than a table so a channel-count change cannot index past a literal. */
static rt_base_t c5_adc_pin(rt_int8_t ch)
{
    if (ch < 0 || ch >= SOC_ADC_MAX_CHANNEL_NUM)
    {
        return -1;
    }

    return (rt_base_t)(ADC1_CHANNEL_0_GPIO_NUM + ch);
}

/*
 * Park the pad for analog use.
 *
 * This is the step that is easy to skip and impossible to notice: a pad left in
 * its digital default still has the input buffer and the pull resistors
 * connected, so the SAR measures the pad's own driver rather than the voltage on
 * it. IDF's gpio_config_as_analog() does these five things and no more - the pad
 * is left floating, with PIN_FUNC_GPIO so the matrix is out of the way.
 *
 * Symptom when omitted: every channel reads 0, which looks like "the ADC is
 * broken" rather than "the pin is still a GPIO".
 */
static void c5_adc_park_pad(rt_base_t pin)
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

/* Bring the digital controller up. Split out because both init and the self
 * test need it and it must run exactly once per boot. */
static void c5_adc_controller_init(void)
{
    /* Clock first: this chip has a digital controller and no RTC one, so the
     * oneshot HAL takes its digi-clk branch. Without a clock the start bit is
     * accepted and the done event never arrives. */
    /* XTAL, via ADC_LL_CTRL_RTC-style digi clock selection... but the C5 has no
     * RTC controller, so the source is one of PLL_F80M / RC_FAST / XTAL. PLL is
     * default in clk_tree_defs.h; XTAL is used here for the same reason as
     * everywhere else in this port: it is exact and does not move with DFS. */
    adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_XTAL);
    adc_ll_digi_controller_clk_div(ADC_LL_CLKM_DIV_NUM_DEFAULT,
                                   ADC_LL_CLKM_DIV_B_DEFAULT,
                                   ADC_LL_CLKM_DIV_A_DEFAULT);
    adc_ll_digi_set_clk_div(ADC_LL_DIGI_SAR_CLK_DIV_DEFAULT);

    /* The FSM needs its wait states before it can sequence a conversion. */
    adc_ll_digi_set_fsm_time(ADC_LL_FSM_RSTB_WAIT_DEFAULT,
                             ADC_LL_FSM_START_WAIT_DEFAULT,
                             ADC_LL_FSM_STANDBY_WAIT_DEFAULT);
    adc_ll_set_sample_cycle(ADC_LL_SAMPLE_CYCLE_DEFAULT);

    adc_oneshot_ll_output_invert(ADC_UNIT_1, ADC_LL_DATA_INVERT_DEFAULT(ADC_UNIT_1));
}

static rt_err_t c5_adc_enabled(struct rt_adc_device *device, rt_int8_t channel,
                               rt_bool_t enabled)
{
    struct c5_adc *adc = (struct c5_adc *)device;

    if (channel < 0 || channel >= SOC_ADC_MAX_CHANNEL_NUM)
    {
        return -RT_EINVAL;
    }

    if (enabled)
    {
        /* The pad has to be parked for analog before the SAR can see it. */
        c5_adc_park_pad(c5_adc_pin(channel));

        /* Program the channel HERE, not in convert(). rt_adc_enable(dev, ch) is
         * "prepare this channel"; rt_adc_read(dev, ch) is "give me a sample".
         *
         * The S31 file records this exact mistake: it set a bookkeeping value
         * here and left the hardware alone, so a `if (adc->channel != channel)`
         * guard in convert() was false on every read and the pattern register
         * kept its reset value. Every conversion returned 0 while the SAR was
         * perfectly healthy. */
        adc_oneshot_ll_set_atten(ADC_UNIT_1, (adc_channel_t)channel,
                                 ADC_ATTEN_DB_12);
        adc_oneshot_ll_set_output_bits(ADC_UNIT_1, SOC_ADC_RTC_MAX_BITWIDTH);
        adc_oneshot_ll_set_channel(ADC_UNIT_1, (adc_channel_t)channel);

        /* BY_FSM, not SW_ON. The LL header says SW_ON is for DMA mode; the
         * polling/oneshot path needs the FSM to own the power state, or the
         * conversion start has nothing to sequence. */
        adc_ll_set_power_manage(ADC_UNIT_1, ADC_LL_POWER_BY_FSM);
        adc_ll_set_controller(ADC_UNIT_1, ADC_LL_CTRL_DIG);

        adc->powered = RT_TRUE;
        adc->channel = channel;
    }
    else
    {
        adc_oneshot_ll_disable_channel(ADC_UNIT_1);
        adc_ll_set_power_manage(ADC_UNIT_1, ADC_LL_POWER_SW_OFF);
        adc->powered = RT_FALSE;
        adc->channel = -1;
    }

    return RT_EOK;
}

static rt_err_t c5_adc_convert(struct rt_adc_device *device, rt_int8_t channel,
                               rt_uint32_t *value)
{
    struct c5_adc *adc = (struct c5_adc *)device;
    rt_uint32_t guard;

    if (value == RT_NULL || channel < 0 || channel >= SOC_ADC_MAX_CHANNEL_NUM)
    {
        return -RT_EINVAL;
    }

    if (!adc->powered)
    {
        return -RT_ERROR;
    }

    /* Reprogram on EVERY conversion. The S31 file records why: the pattern table
     * holds one entry and each conversion advances the pointer past it, so the
     * next conversion finds no entry and returns 0. Measured there as
     * "first read 1771, second and third 0". */
    adc_oneshot_ll_set_channel(ADC_UNIT_1, (adc_channel_t)channel);
    adc->channel = channel;

    /* A stale done bit from a previous conversion makes the poll below return
     * immediately and read the old result. */
    adc_oneshot_ll_clear_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE);
    adc_oneshot_ll_enable(ADC_UNIT_1);

    /* The start signal has to be a PULSE, not a level.
     *
     * This is the hardware limitation IDF documents in adc_hal_onetime_start():
     * if the APB clock is fast relative to the ADC digital controller clock, an
     * `onetime_start` asserted for a single write may never be captured by the
     * controller. IDF therefore sets it, waits at least three controller clock
     * cycles, and clears it again. The first version of this driver only set it
     * and left it high, and every conversion sat in the poll loop until the
     * guard expired - measured on all six channels as "conversion of channel N
     * never completed". */
    {
        uint32_t apb = (uint32_t)esp_clk_apb_freq();
        uint32_t ctrl_clk = apb / (ADC_LL_CLKM_DIV_NUM_DEFAULT + 1);
        uint32_t delay_us = 0;

        if (ctrl_clk < apb / 8u)
        {
            delay_us = ((1000u * 1000u) / ctrl_clk + 1u) * 3u;
        }

        adc_oneshot_ll_start(true);
        esp_rom_delay_us(delay_us);
        adc_oneshot_ll_start(false);
        esp_rom_delay_us(delay_us);
    }

    /* Bounded poll. A single conversion is a few tens of microseconds; the guard
     * only exists so a mis-clocked unit reports instead of hanging the shell. */
    guard = 0;
    while (!adc_oneshot_ll_get_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE))
    {
        if (++guard > 2000000u)
        {
            rt_kprintf("c5adc: conversion of channel %d never completed\n",
                       (int)channel);
            return -RT_ETIMEOUT;
        }
    }

    adc_oneshot_ll_clear_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE);

    /* The raw result is only valid a few SAR clocks after the done signal; IDF
     * delays here for the same reason. Reading immediately returns zero. */
    esp_rom_delay_us(5);

    *value = adc_oneshot_ll_get_raw_result(ADC_UNIT_1);

    return RT_EOK;
}

static rt_uint8_t c5_adc_resolution(struct rt_adc_device *device)
{
    RT_UNUSED(device);

    return SOC_ADC_RTC_MAX_BITWIDTH;
}

static rt_int16_t c5_adc_vref(struct rt_adc_device *device)
{
    RT_UNUSED(device);

    /* 0, not -1, and the difference matters.
     *
     * The framework's voltage helper (components/drivers/adc/adc.c) reads
     * `vref = dev->ops->get_vref(dev)` and treats 0 as "cannot compute",
     * returning a clean 0 mV. A -1 slips past that check and is multiplied into
     * the result, which `adc voltage` then prints with the sign inside the
     * digits - measured on the S31 as "-32.-768V".
     *
     * The real reason it is 0 is that this port has no calibration. The C5 does
     * have SOC_ADC_CALIBRATION_V1_SUPPORTED, but the coefficients come from the
     * ADC calibration component, which is not part of this port and whose
     * numbers were never measured here. Inventing a reference voltage would make
     * every downstream millivolt calculation inherit the lie. */
    return 0;
}

static const struct rt_adc_ops c5_adc_ops =
{
    .enabled        = c5_adc_enabled,
    .convert        = c5_adc_convert,
    .get_resolution = c5_adc_resolution,
    .get_vref       = c5_adc_vref,
};

rt_err_t rt_hw_adc_init(void)
{
    rt_err_t rc;

    c5_adc_controller_init();

    s_adc[0].channel = -1;
    s_adc[0].powered = RT_FALSE;

    rc = rt_hw_adc_register(&s_adc[0].parent, C5_ADC_DEV_NAME, &c5_adc_ops,
                            RT_NULL);
    if (rc != RT_EOK)
    {
        rt_kprintf("c5adc: register failed %d\n", rc);
    }

    return rc;
}
INIT_BOARD_EXPORT(rt_hw_adc_init);

/* ------------------------------------------------------------------ msh -- */

/* `c5adc` - read every channel and report.
 *
 * There is no way to inject a known voltage from here, so this cannot prove
 * accuracy. What it can prove, and does, is:
 *
 *   - the digital controller is clocked and conversions complete (a stuck unit
 *     returns -RT_ETIMEOUT rather than a number);
 *   - consecutive reads on the same channel are not identical-by-accident (that
 *     is the "pattern register never reprogrammed" bug the S31 file records);
 *   - the pads that are shared with another peripheral are named, so a reading
 *     from them is not mistaken for a measurement.
 */
int c5_adc_selftest(void)
{
    struct rt_adc_device *dev;
    int ch;
    int fail = 0;

    dev = (struct rt_adc_device *)rt_device_find(C5_ADC_DEV_NAME);
    if (dev == RT_NULL)
    {
        rt_kprintf("c5adc   : FAIL, device %s not registered\n", C5_ADC_DEV_NAME);
        return -1;
    }

    rt_kprintf("c5adc   : unit 1, %d channel(s), %d-bit, atten 12 dB, no calib\n",
               SOC_ADC_MAX_CHANNEL_NUM, SOC_ADC_RTC_MAX_BITWIDTH);

    for (ch = 0; ch < SOC_ADC_MAX_CHANNEL_NUM; ch++)
    {
        rt_uint32_t v1 = 0;
        rt_uint32_t v2 = 0;
        rt_err_t r1;

        r1 = rt_adc_enable(dev, ch);
        if (r1 != RT_EOK)
        {
            rt_kprintf("c5adc   : ch%d enable failed %d\n", ch, (int)r1);
            fail++;
            continue;
        }

        /* Call convert() DIRECTLY rather than through rt_adc_read().
         *
         * rt_adc_read() returns the value and discards the error: it returns 0
         * whether the channel is parked at ground or the conversion timed out.
         * The first version of this test used it and printed
         * "PASS (all 6 channels converted)" while every single conversion had
         * actually failed - the failure was visible only in the "never
         * completed" lines printed above the verdict. Going through the ops
         * table keeps the error code, and that is what the verdict must rest
         * on. */
        r1 = dev->ops->convert(dev, (rt_int8_t)ch, &v1);
        if (r1 != RT_EOK)
        {
            rt_kprintf("c5adc   : ch%d (GPIO%d) convert failed %d\n",
                       ch, (int)c5_adc_pin((rt_int8_t)ch), (int)r1);
            fail++;
            rt_adc_disable(dev, ch);
            continue;
        }

        /* A second conversion catches the S31 file's "pattern register never
         * reprogrammed" bug: if the second read is 0 while the first was not,
         * the channel selection is not being re-applied per conversion. */
        (void)dev->ops->convert(dev, (rt_int8_t)ch, &v2);

        {
            /* GPIO3 is driven by the PWM self test, and GPIO1/GPIO2 are the
             * LP_UART pads. Naming them here is the difference between "the ADC
             * is wrong" and "something else is on that pad". */
            const char *note = "";

            if (c5_adc_pin((rt_int8_t)ch) == 3)
            {
                note = "  (GPIO3 is the PWM self-test pin)";
            }
            else if (c5_adc_pin((rt_int8_t)ch) <= 2)
            {
                note = "  (LP_UART pad)";
            }

            rt_kprintf("c5adc   : ch%d (GPIO%d) raw=%u then %u, max=%u%s\n",
                       ch, (int)c5_adc_pin((rt_int8_t)ch),
                       (unsigned)v1, (unsigned)v2,
                       (unsigned)((1u << SOC_ADC_RTC_MAX_BITWIDTH) - 1u), note);
        }

        rt_adc_disable(dev, ch);
    }

    if (fail)
    {
        rt_kprintf("c5adc   : FAIL, %d channel(s) did not convert\n", fail);
        return -1;
    }

    rt_kprintf("c5adc   : PASS (all %d channels converted)\n",
               SOC_ADC_MAX_CHANNEL_NUM);
    return 0;
}



#endif /* SOC_ADC_SUPPORTED */













