/*
 * ESP32-C5 board resource registry: who owns which pin.
 *
 * This is not a driver. It exists because of a class of failure that has already
 * cost this port two rounds and would cost more:
 *
 *   - `c5adc` parks its channel pads for analog. ADC1 channel 3 is GPIO4, and
 *     GPIO4 is where the CAN self test puts TWAI0's TX and RX. Running `c5adc`
 *     before `c5twai` in one session made CAN fail with a form error at every
 *     rate, while `c5twai` alone passed (搂17d).
 *   - The native USB serial/JTAG pads are GPIO13/14, which is where the PCNT and
 *     MCPWM self tests happened to be pointed.
 *
 * Neither is a bug in a driver. Both are two drivers wanting one pad, discovered
 * only by running them in an unlucky order. So the pin assignment lives in ONE
 * table here, each driver keeps its own #define (so a single driver can still be
 * built and read on its own), and `c5pins` reports:
 *
 *   - every pad in use and by whom,
 *   - every pad claimed twice,
 *   - every pad that a peripheral has a fixed function on but a test is using
 *     anyway (USB, flash, console).
 *
 * The table is checked against the drivers by construction, not by hope: each row
 * names the header and macro it mirrors, so a drift shows up as a comment that no
 * longer matches rather than as a silent conflict.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"

#include "soc/gpio_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"

#include "port_res.h"

typedef struct
{
    rt_int16_t  pin;
    const char *owner;
    const char *macro;      /* the #define this row mirrors */
} c5_pin_use_t;

/*
 * Every pad this port touches, in one place.
 *
 * Kept as a static table rather than gathered from the drivers at run time
 * because most of these are compile-time #defines, not variables - there is no
 * symbol to read. The `macro` column is what keeps it honest: when a driver's pin
 * changes, this table is visibly stale instead of quietly wrong.
 */
static const c5_pin_use_t s_pins[] =
{
    /* Console. Fixed by the UART0 IOMUX default and by the flashing setup. */
    { 11, "uart0 console TX",           "UART0 default" },
    { 12, "uart0 console RX",           "UART0 default" },

    /* Self-test and device pins, one row per driver. */
    {  0, "RMT test (tx pad)",          "C5_RMT_TEST_PIN" },
    {  1, "MCPWM output A",             "C5_MCPWM_TEST_PIN_A" },
    {  2, "SPI MISO",                   "C5_SPI0_MISO_PIN" },
    {  3, "LEDC PWM test",              "C5_PWM_TEST_PIN" },
    {  4, "CAN TX and RX (one pad)",     "C5_TWAI_TX_PIN" },
    {  6, "SPI clock",                  "C5_SPI0_SCLK_PIN" },
    {  7, "SPI MOSI",                   "C5_SPI0_MOSI_PIN" },
    {  8, "I2C SCL",                    "C5_I2C0_SCL_PIN" },
    {  9, "I2C SDA",                    "C5_I2C0_SDA_PIN" },
    { 10, "SPI chip select",            "C5_SPI0_CS_PIN" },
    {  5, "PCNT test",                  "C5_PCNT_TEST_PIN" },

    /* Multi-purpose pads that a peripheral has a fixed function on. Listed so the
     * check below can flag a test that strays onto one. */
    { -1, RT_NULL, RT_NULL }
};

/* Pads with a fixed non-GPIO function on this package. */
static const c5_pin_use_t s_fixed[] =
{
    { 13, "USB D- (native USB serial/JTAG)", "USB_INT_PHY0_DM_GPIO_NUM" },
    { 14, "USB D+ (native USB serial/JTAG)", "USB_INT_PHY0_DP_GPIO_NUM" },
    { 15, "in-package flash",                "SPI flash IOMUX" },
    { 16, "in-package flash",                "SPI flash IOMUX" },
    { 17, "in-package flash",                "SPI flash IOMUX" },
    { 18, "in-package flash",                "SPI flash IOMUX" },
    { 19, "in-package flash",                "SPI flash IOMUX" },
    { 20, "in-package flash",                "SPI flash IOMUX" },
    { 21, "in-package flash",                "SPI flash IOMUX" },
    { 22, "in-package flash",                "SPI flash IOMUX" },
    { -1, RT_NULL, RT_NULL }
};

/* ADC1 channel-to-pad mapping, from soc/adc_channel.h. Listed because the ADC
 * driver parks its pads for analog and that is what broke CAN on IO4. */
static const rt_int16_t s_adc_pin[6] = { 1, 2, 3, 4, 5, 6 };

/* ------------------------------------------------------------------ msh -- */

/* `c5pins` - report the pin map, the conflicts, and the fixed-function clashes.
 *
 * Returns 0 when nothing is wrong, -1 when a pad is claimed twice or a test is
 * sitting on a fixed-function pad. That return value is the point: this is meant
 * to be runnable from a boot self-check, not only read by a human.
 */
/* `c5probe [first] [last]` - which pads actually respond on this package.
 *
 * SOC_GPIO_PIN_COUNT is 29, but a pad that the package does not bring out reads
 * back as its pull state no matter what is written to it, and driving an
 * unconnected pad looks identical to driving a working one. The only way to tell
 * them apart without a meter is to drive a level and read it back through the
 * input path: a pad that exists follows, a pad that does not never will.
 *
 * This is what decides where a driver can be placed. The C5's LP/RTC pads are
 * only GPIO0..GPIO6 (rtc_io_periph.c), and this port already uses all of 0..6,
 * so an LP peripheral needs pads freed from here. */
int c5_probe_pads(int first, int last)
{
    int pin;
    int found = 0;

    if (first < 0 || last < first || last >= SOC_GPIO_PIN_COUNT)
    {
        first = 0;
        last = SOC_GPIO_PIN_COUNT - 1;
    }

    rt_kprintf("c5probe : driving each pad and reading it back\n");

    for (pin = first; pin <= last; pin++)
    {
        rt_uint32_t hi;
        rt_uint32_t lo;

        gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
        esp_rom_gpio_connect_out_signal((uint32_t)pin, SIG_GPIO_OUT_IDX, false, false);
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin, false, false);
        gpio_ll_output_enable(&GPIO, (uint8_t)pin);
        gpio_ll_input_enable(&GPIO, (uint8_t)pin);
        gpio_ll_pulldown_dis(&GPIO, (uint8_t)pin);
        gpio_ll_pullup_dis(&GPIO, (uint8_t)pin);

        gpio_ll_set_level(&GPIO, (uint8_t)pin, 1);
        hi = gpio_ll_get_level(&GPIO, (uint8_t)pin);
        gpio_ll_set_level(&GPIO, (uint8_t)pin, 0);
        lo = gpio_ll_get_level(&GPIO, (uint8_t)pin);

        if (hi == 1u && lo == 0u)
        {
            found++;
            rt_kprintf("c5probe : IO%-2d follows (1 -> 1, 0 -> 0)\n", pin);
        }
        else
        {
            rt_kprintf("c5probe : IO%-2d does NOT follow (1 -> %u, 0 -> %u)\n",
                       pin, (unsigned)hi, (unsigned)lo);
        }

        gpio_ll_output_disable(&GPIO, (uint8_t)pin);
    }

    rt_kprintf("c5probe : %d of %d pad(s) respond\n", found, last - first + 1);
    return 0;
}

int c5_pins_report(void)
{
    int n_pins = (int)(sizeof(s_pins) / sizeof(s_pins[0])) - 1;    /* drop sentinel */
    int n_fixed = (int)(sizeof(s_fixed) / sizeof(s_fixed[0])) - 1;
    int problems = 0;
    int i;
    int j;

    rt_kprintf("c5pins  : %d pad(s) in use\n", n_pins);

    for (i = 0; i < n_pins; i++)
    {
        rt_kprintf("c5pins  : IO%-2d  %-28s (%s)\n",
                   (int)s_pins[i].pin, s_pins[i].owner, s_pins[i].macro);
    }

    /* 1. The same pad claimed by two owners. */
    for (i = 0; i < n_pins; i++)
    {
        for (j = i + 1; j < n_pins; j++)
        {
            if (s_pins[i].pin == s_pins[j].pin)
            {
                rt_kprintf("c5pins  : CONFLICT IO%d claimed by \"%s\" and \"%s\"\n",
                           (int)s_pins[i].pin, s_pins[i].owner, s_pins[j].owner);
                problems++;
            }
        }
    }

    /* 2. A test pin sitting on a pad that already has a fixed function. */
    for (i = 0; i < n_pins; i++)
    {
        for (j = 0; j < n_fixed; j++)
        {
            if (s_pins[i].pin == s_fixed[j].pin)
            {
                rt_kprintf("c5pins  : FIXED-FUNCTION CLASH IO%d: \"%s\" wants it, "
                           "but it is %s\n",
                           (int)s_pins[i].pin, s_pins[i].owner, s_fixed[j].owner);
                problems++;
            }
        }
    }

    /* 3. Which ADC channels share a pad with a driver. The ADC parks its pad for
     * analog, which is exactly what broke CAN - so this is the check that would
     * have caught it. */
    rt_kprintf("c5pins  : ADC1 channels and the drivers that share their pads:\n");
    for (i = 0; i < 6; i++)
    {
        const char *who = RT_NULL;

        for (j = 0; j < n_pins; j++)
        {
            if (s_pins[j].pin == s_adc_pin[i])
            {
                who = s_pins[j].owner;
                break;
            }
        }

        if (who == RT_NULL)
        {
            rt_kprintf("c5pins  :   ch%d IO%-2d free\n", i, (int)s_adc_pin[i]);
        }
        else
        {
            rt_kprintf("c5pins  :   ch%d IO%-2d SHARED with %s - reading it parks "
                       "the pad for analog\n", i, (int)s_adc_pin[i], who);
            /* Not counted as a problem: reading an ADC channel on a shared pad is
             * legal, it just has a side effect. The point is that it is stated. */
        }
    }

    if (problems == 0)
    {
        rt_kprintf("c5pins  : PASS, no pad is claimed twice and no test sits on a "
                   "fixed-function pad\n");
        return 0;
    }

    rt_kprintf("c5pins  : FAIL, %d problem(s) above\n", problems);
    return -1;
}


