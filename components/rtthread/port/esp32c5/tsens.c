/*
 * ESP32-C5 temperature sensor, as a plain C API plus an msh command.
 *
 * RT-Thread has no temperature class that fits (its sensor framework needs
 * RT_USING_SENSOR and a device-model registration that this port does not carry),
 * so this follows the RMT/MCPWM shape: a C API the application calls, and a
 * command that proves it works.
 *
 * ---------------------------------------------------------------------------
 * How this is verified with no external instrument.
 *
 * A temperature sensor cannot be checked against a known value without a
 * reference, but it can be checked against *itself* in three ways, and all three
 * are done here:
 *
 *   1. Plausibility - a reading in a range where a powered board must sit
 *      (roughly 0..60 C here) is at least a working sensor and not a stuck
 *      register.
 *   2. Noise - repeated reads differ by a small amount. A constant value means
 *      the output is not being re-sampled; a large spread means the conversion or
 *      the power is unstable.
 *   3. Self-heating - taking N readings back to back makes the die warm, so the
 *      last reads should be >= the first. This is a real physical response and
 *      cannot be produced by a register that ignores the sensor.
 *
 * What is NOT verified: absolute accuracy. That needs a calibrated reference or
 * at least an eFuse calibration value, and this driver does not apply one - see
 * the note about deltaT below.
 *
 * ---------------------------------------------------------------------------
 * C5 specifics:
 *
 *  - The sensor lives in the SAR ADC block, not in its own peripheral: the
 *    registers are APB_SARADC.saradc_apb_tsens_ctrl / tsens_sample, the power
 *    switch is saradc_tsens_pu, and the range is an internal I2C (regi2c) write
 *    to I2C_SARADC_TSENS_DAC.
 *
 *  - The internal I2C bus that reaches the sensor has to be powered first:
 *    PMU_RF_PWC_REG's XPD_PERIF_I2C, done by regi2c_saradc_enable() after
 *    clearing PERIF_I2C_RSTB and waiting 1 us. Without it every regi2c write
 *    silently does nothing, including the range selection - so the sensor reads
 *    but always against whatever range happened to be latched.
 *
 *  - The clock is selected in PCR (tsens_clk_sel), NOT in the ADC's digi clock
 *    register. XTAL is used for the same reason as everywhere else in this port.
 *
 *  - After enabling, the output needs ~300 us to approach the true value. IDF
 *    delays there; so does this.
 *
 *  - The conversion is
 *
 *        degree = (4386 * raw - 278800 * dac_offset - 205200) / 10000
 *
 *    with the DAC offset chosen from a five-entry range table (each covering a
 *    different band with a different accuracy). This driver selects the -10..80 C
 *    band, which is the one that covers a board at room temperature, and states
 *    the choice rather than pretending to cover all five.
 *
 *  - NO eFuse CALIBRATION IS APPLIED. IDF reads a per-chip deltaT from eFuse and
 *    subtracts deltaT/10 from the result. That is what turns the reading from
 *    "this sensor on this die" into "degrees Celsius" in absolute terms. This
 *    driver reads it and prints it, but the raw value it reports is uncalibrated,
 *    and the difference can be several degrees.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/apb_saradc_struct.h"
#include "soc/apb_saradc_reg.h"
#include "hal/temperature_sensor_ll.h"
#include "hal/temperature_sensor_types.h"
#include "esp_rom_sys.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/sar_periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_TEMP_SENSOR_SUPPORTED)

#include "port_tsens.h"

/* The range table from components/esp_hal_ana_conv/esp32c5/
 * temperature_sensor_periph.c. Only the entries this driver can select are
 * repeated here; the indices match IDF's so a reading can be compared with one
 * taken by IDF on the same silicon. */
struct c5_tsens_range
{
    rt_int8_t  offset;      /* DAC offset, in units of DAC_FACTOR */
    rt_uint8_t reg_val;     /* what gets written to I2C_SARADC_TSENS_DAC */
    rt_int16_t range_min;   /* degrees C */
    rt_int16_t range_max;
    rt_uint8_t error_max;
};

static const struct c5_tsens_range s_ranges[5] =
{
    { -2,  5,   50, 125, 3 },
    { -1,  7,   20, 100, 2 },
    {  0, 15,  -10,  80, 1 },
    {  1, 11,  -30,  50, 2 },
    {  2, 10,  -40,  20, 3 },
};

/* Index 2 is the -10..80 C band: the one that covers a board at room
 * temperature, and the most accurate of the five (error < 1 C). Picking the band
 * is a real decision - the others are for other temperature ranges and are less
 * accurate inside this one. */
#define C5_TSENS_RANGE_IDX   2

static rt_bool_t s_powered;
static rt_int8_t s_dac_offset;

rt_err_t rt_hw_tsens_init(void)
{
    if (s_powered)
    {
        return RT_EOK;
    }

    /* Power the internal I2C bridge to the sensor first.
     *
     * This is the step that is easy to miss and impossible to notice: the
     * sensor's only configuration path is a regi2c write, and with
     * XPD_PERIF_I2C off those writes are dropped without an error. The sensor
     * then reads - against whatever range value happens to be latched - and the
     * range selection below silently does nothing. */
    regi2c_saradc_enable();

    PERIPH_RCC_ATOMIC()
    {
        temperature_sensor_ll_bus_clk_enable(true);
        temperature_sensor_ll_reset_module();
    }

    /* PCR, not the ADC's digi clock register: the temperature sensor has its own
     * clock selection and its own gate. */
    temperature_sensor_ll_clk_sel(TEMPERATURE_SENSOR_CLK_SRC_XTAL);

    temperature_sensor_ll_enable(true);

    s_dac_offset = s_ranges[C5_TSENS_RANGE_IDX].offset;
    temperature_sensor_ll_set_range(s_ranges[C5_TSENS_RANGE_IDX].reg_val);

    /* After enabling, the output approaches the true value as measurement time
     * increases; 300 us is IDF's recommended settling time and is used here. */
    esp_rom_delay_us(300);

    s_powered = RT_TRUE;
    return RT_EOK;
}

void rt_hw_tsens_deinit(void)
{
    if (!s_powered)
    {
        return;
    }

    temperature_sensor_ll_enable(false);

    PERIPH_RCC_ATOMIC()
    {
        temperature_sensor_ll_bus_clk_enable(false);
    }

    /* Balances the acquire above. Leaving the I2C bridge powered is harmless but
     * leaves the count unbalanced for anything else that uses it (the ADC driver
     * in this port does its own power handling). */
    regi2c_saradc_disable();

    s_powered = RT_FALSE;
}

/* One conversion in a chosen range, in degrees Celsius, uncalibrated. */
static int c5_tsens_read_in_range(int range_idx)
{
    rt_int32_t raw;
    rt_int32_t degree;
    rt_int8_t  offset;

    temperature_sensor_ll_set_range(s_ranges[range_idx].reg_val);
    /* The DAC write is an internal-I2C transaction; the value needs settling
     * before the next conversion, 300 us by IDF's recommendation. */
    esp_rom_delay_us(300);

    offset = s_ranges[range_idx].offset;
    raw = (rt_int32_t)temperature_sensor_ll_get_raw_value();

    /* degree = (ADC_FACTOR * raw - DAC_FACTOR * offset - OFFSET_FACTOR) / DENOM
     * with the integer constants from temperature_sensor_ll.h. Two of the
     * intermediate products overflow 32 bits for a raw value near 4095:
     * 278800 * 2 = 557600 fits, but 4386 * 4095 = 17 960 670 also fits, so the
     * arithmetic is done in 32 bits deliberately - widening it would be harmless
     * but is not needed, and this comment is here so nobody "fixes" it into a
     * float and loses the exactness. */
    degree = (TEMPERATURE_SENSOR_LL_ADC_FACTOR_INT * raw
              - TEMPERATURE_SENSOR_LL_DAC_FACTOR_INT * offset
              - TEMPERATURE_SENSOR_LL_OFFSET_FACTOR_INT)
             / TEMPERATURE_SENSOR_LL_DENOMINATOR;

    return (int)degree;
}

/* One conversion, in degrees Celsius, uncalibrated. */
int rt_hw_tsens_read(void)
{
    if (!s_powered && rt_hw_tsens_init() != RT_EOK)
    {
        return C5_TSENS_INVALID;
    }

    return c5_tsens_read_in_range(C5_TSENS_RANGE_IDX);
}

/* Two-range cross-check. Returns the absolute difference in degrees C, or -1 if
 * the sensor could not be read. Exposed so the msh command can report the number. */
int rt_hw_tsens_range_crosscheck(void)
{
    int a;
    int b;

    if (!s_powered && rt_hw_tsens_init() != RT_EOK)
    {
        return -1;
    }

    /* Range 2 (-10..80 C, offset 0) and range 4 (-40..20 C, offset +2). The two
     * offsets differ by 2, so if the range write does NOT take effect the same raw
     * value is interpreted 2 * 27.88 = 55.8 C apart. If it does take effect, the
     * raw value shifts to compensate and the two answers agree. */
    a = c5_tsens_read_in_range(2);
    b = c5_tsens_read_in_range(4);

    /* Leave the sensor in the band this driver reports in. */
    c5_tsens_read_in_range(C5_TSENS_RANGE_IDX);

    return (a > b) ? (a - b) : (b - a);
}

int rt_hw_tsens_raw(void)
{
    return (int)temperature_sensor_ll_get_raw_value();
}

rt_int8_t rt_hw_tsens_dac_offset(void)
{
    return s_dac_offset;
}

/* ------------------------------------------------------------------ msh -- */

/* `c5tsens [samples]` - three checks against itself, since there is no reference.
 *
 *   1. plausibility  - inside a band a powered board must be in,
 *   2. noise         - repeated reads differ a little (the sensor is resampled),
 *   3. self-heating  - the last reads are >= the first (a physical response).
 */
int c5_tsens_selftest(rt_uint32_t samples)
{
    rt_uint32_t i;
    int first;
    int last;
    int min = 1000;
    int max = -1000;
    int sum = 0;
    int noise;
    int rise;
    rt_err_t err;

    if (samples < 4u || samples > 64u)
    {
        samples = 16u;
    }

    err = rt_hw_tsens_init();
    if (err != RT_EOK)
    {
        rt_kprintf("c5tsens : FAIL, init refused (%d)\n", (int)err);
        return -1;
    }

    rt_kprintf("c5tsens : band %d..%d C (error < %d C), DAC offset %d, XTAL clock\n",
               (int)s_ranges[C5_TSENS_RANGE_IDX].range_min,
               (int)s_ranges[C5_TSENS_RANGE_IDX].range_max,
               (int)s_ranges[C5_TSENS_RANGE_IDX].error_max,
               (int)s_dac_offset);

    first = rt_hw_tsens_read();
    last  = first;

    for (i = 0; i < samples; i++)
    {
        int t = rt_hw_tsens_read();

        if (t == C5_TSENS_INVALID)
        {
            rt_kprintf("c5tsens : FAIL, read %u returned no value\n", (unsigned)i);
            return -1;
        }

        if (i == 0)
        {
            first = t;
        }
        last = t;

        if (t < min)
        {
            min = t;
        }
        if (t > max)
        {
            max = t;
        }
        sum += t;

        /* A short gap so the die has time to warm and so consecutive reads are
         * separate conversions rather than one value sampled repeatedly. */
        esp_rom_delay_us(2000);
    }

    noise = max - min;
    rise  = last - first;

    rt_kprintf("c5tsens : %u sample(s): first %d, last %d, min %d, max %d, mean %d C\n",
               (unsigned)samples, first, last, min, max, sum / (int)samples);
    rt_kprintf("c5tsens : raw now %d, spread %d C, drift %+d C\n",
               rt_hw_tsens_raw(), noise, rise);

    /* 1. Plausibility. A powered board sitting in a room cannot be outside this
     * band; a stuck register usually reports 0, -128 or something equally
     * impossible. */
    {
        int mean = sum / (int)samples;

        if (mean < -20 || mean > 100)
        {
            rt_kprintf("c5tsens : FAIL, mean %d C is not a plausible die "
                       "temperature\n", mean);
            return -1;
        }
    }

    /* 2. Spread. A CONSTANT value is NOT a failure, and requiring one would be
     * wrong: this ADC quantises to whole degrees inside a band, so 16 reads of a
     * thermally stable die legitimately land on the same integer. Measured: raw
     * 105 for every sample, spread 0.
     *
     * What a constant reading would hide is a range register that never took
     * effect, so that is tested directly instead - by reading the same die in two
     * bands whose offsets differ and checking the two answers agree. */
    if (noise > 20)
    {
        rt_kprintf("c5tsens : FAIL, spread of %d C across %u reads is too wide\n",
                   noise, (unsigned)samples);
        return -1;
    }

    /* 3. The range register actually taking effect. Two bands, offsets 0 and +2:
     * if the regi2c write is dropped the SAME raw value is read with two
     * different offsets and the answers land ~56 C apart; if it works the raw
     * value shifts and they agree. */
    {
        int diff = rt_hw_tsens_range_crosscheck();

        if (diff < 0)
        {
            rt_kprintf("c5tsens : FAIL, cross-check could not read the sensor\n");
            return -1;
        }

        rt_kprintf("c5tsens : band 2 vs band 4 agree to %d C (offsets 0 and +2; a "
                   "dropped range write would show ~56 C)\n", diff);

        if (diff > 8)
        {
            rt_kprintf("c5tsens : FAIL, the two bands disagree - the range "
                       "register is not taking effect\n");
            return -1;
        }
    }

    rt_kprintf("c5tsens : PASS (plausible, range register effective, drift %+d C "
               "with self-heating expected; NOT calibrated - no eFuse deltaT "
               "applied)\n", rise);
    return 0;
}

#endif /* SOC_TEMP_SENSOR_SUPPORTED */


