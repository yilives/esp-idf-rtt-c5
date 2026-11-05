/*
 * ESP32-S31 GPIO driver: rt_pin_ops over the GPIO LL.
 *
 * Same reasoning as the UART driver - esp_driver_gpio/src/gpio.c itself is
 * FreeRTOS free, but it pulls in the interrupt allocator and the caps layers,
 * and the LL is both smaller and directly auditable. Everything below touches
 * IO_MUX and the GPIO registers only.
 *
 * Two S31 specific points worth remembering:
 *
 * 1. The pad must be routed to the GPIO peripheral before input/output enable
 *    has any effect: IO_MUX.gpio[n].mcu_sel has to be PIN_FUNC_GPIO. This is
 *    step one of what IDF's gpio_input_enable()/gpio_output_enable() do.
 *
 * 2. S31 has four independent GPIO interrupt channels (ETS_GPIO_INTR0..3),
 *    a new feature compared to C3/C6. gpio_ll_intr_enable_on_core() hardcodes
 *    INTR0 and carries an IDF TODO (IDF-7995) for the rest, so this driver
 *    uses channel 0 -> ETS_GPIO_INUM. All pins share one handler, which is
 *    exactly what the RT-Thread pin framework expects.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/interrupts.h"
#include "hal/gpio_ll.h"
#include "esp_attr.h"
#include "riscv/interrupt.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"

#include "soc/reg_base.h"

#include "port_gpio.h"
#include "port_intr.h"

/* S31 exposes 62 pads; GPIO29 and GPIO41 are consumed by the SPI flash. */
#define S31_PIN_NUM             SOC_GPIO_PIN_COUNT

struct s31_pin_irq
{
    void      (*hdr)(void *args);
    void       *args;
    rt_uint8_t  mode;       /* PIN_IRQ_MODE_* */
    rt_bool_t   attached;
};

static struct s31_pin_irq s_pin_irq[S31_PIN_NUM];
static rt_bool_t            s_gpio_isr_ready;

static rt_bool_t s31_pin_valid(rt_base_t pin)
{
    if (pin < 0 || pin >= S31_PIN_NUM)
    {
        return RT_FALSE;
    }

    return ((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) ? RT_TRUE : RT_FALSE;
}

/* Clear the latched interrupt bit of a single pin. The status registers are
 * split at pin 32, and the clear registers are write-1-to-clear. */
static void s31_pin_clear_irq(rt_base_t pin)
{
    if (pin < 32)
    {
        gpio_ll_clear_intr_status(&GPIO, 1U << pin);
    }
    else
    {
        gpio_ll_clear_intr_status_high(&GPIO, 1U << (pin - 32));
    }
}

static void s31_pin_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode)
{
    RT_UNUSED(device);

    if (!s31_pin_valid(pin))
    {
        return;
    }

    /* Detach the pad from whatever peripheral owned it, then start from a
     * clean state: no pulls, no open drain, neither direction enabled. */
    gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
    gpio_ll_pullup_dis(&GPIO, pin);
    gpio_ll_pulldown_dis(&GPIO, pin);
    gpio_ll_od_disable(&GPIO, pin);
    gpio_ll_input_disable(&GPIO, pin);
    gpio_ll_output_disable(&GPIO, pin);

    switch (mode)
    {
    case PIN_MODE_INPUT:
        gpio_ll_input_enable(&GPIO, pin);
        break;

    case PIN_MODE_INPUT_PULLUP:
        gpio_ll_pullup_en(&GPIO, pin);
        gpio_ll_input_enable(&GPIO, pin);
        break;

    case PIN_MODE_INPUT_PULLDOWN:
        gpio_ll_pulldown_en(&GPIO, pin);
        gpio_ll_input_enable(&GPIO, pin);
        break;

    case PIN_MODE_OUTPUT_OD:
        gpio_ll_od_enable(&GPIO, pin);
        /* Open drain still needs the receiver on to read the bus back. */
        gpio_ll_input_enable(&GPIO, pin);
        /* Idle high BEFORE the driver is turned on. The output latch resets to 0,
         * so enabling open drain first and writing a level later holds the bus
         * low from the moment rt_pin_mode() returns - which for I2C/1-Wire looks
         * exactly like a missing pull-up, and every transaction fails. */
        gpio_ll_set_level(&GPIO, pin, 1);
        /* fall through */

    case PIN_MODE_OUTPUT:
        /* Point the output selector at "no peripheral" (SIG_GPIO_OUT_IDX) and
         * hand the output enable to the GPIO register block. Skipping this
         * leaves the pad driven by whichever peripheral was mapped last. */
        gpio_ll_set_output_signal_matrix_source(&GPIO, pin, SIG_GPIO_OUT_IDX, false);
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin, false, false);
        /* Keep the receiver on, exactly as the open-drain case above does.
         *
         * rt_pin_read() is documented as "get the level of the pin", with no
         * restriction to input pins, and it is routine to check an output after
         * writing it. On this SoC the input buffer is off by default, so without
         * this line `pin write 10 high` followed by `pin read 10` reports low -
         * measured, and it is the kind of answer that sends the reader looking for
         * a wiring fault.
         *
         * This was internally inconsistent before: PIN_MODE_OUTPUT_OD enabled the
         * receiver and PIN_MODE_OUTPUT did not, so the same "drive a pin" request
         * behaved differently depending on open-drain. IDF models the distinction
         * explicitly (GPIO_MODE_OUTPUT vs GPIO_MODE_INPUT_OUTPUT); the port now
         * enables both for both, which is what RT-Thread's API assumes. */
        gpio_ll_input_enable(&GPIO, pin);
        gpio_ll_output_enable(&GPIO, pin);
        break;

    default:
        break;
    }
}

static void s31_pin_write(struct rt_device *device, rt_base_t pin, rt_uint8_t value)
{
    RT_UNUSED(device);

    if (!s31_pin_valid(pin))
    {
        return;
    }

    gpio_ll_set_level(&GPIO, pin, value ? 1U : 0U);
}

static rt_ssize_t s31_pin_read(struct rt_device *device, rt_base_t pin)
{
    RT_UNUSED(device);

    if (!s31_pin_valid(pin))
    {
        return -RT_EINVAL;
    }

    return gpio_ll_get_level(&GPIO, pin);
}

/* PIN_IRQ_MODE_* is dense and ordered differently from gpio_int_type_t, so map
 * it instead of relying on the numeric values. */
static const gpio_int_type_t s31_intr_type[] =
{
    [PIN_IRQ_MODE_RISING]         = GPIO_INTR_POSEDGE,
    [PIN_IRQ_MODE_FALLING]        = GPIO_INTR_NEGEDGE,
    [PIN_IRQ_MODE_RISING_FALLING] = GPIO_INTR_ANYEDGE,
    [PIN_IRQ_MODE_HIGH_LEVEL]     = GPIO_INTR_HIGH_LEVEL,
    [PIN_IRQ_MODE_LOW_LEVEL]      = GPIO_INTR_LOW_LEVEL,
};

/* One handler for the whole port: the hardware latches per pin and the
 * dispatcher fans out from the status words.
 *
 * IRAM_ATTR, because this runs from an interrupt: with the cache off (any SPI
 * flash write, or an MSPI/PSRAM operation) code fetched from .flash.text cannot
 * be executed. Measured before the annotation: `s31test irq` produced
 * "Interrupt wdt timeout on CPU0" with MEPC in this function as soon as PSRAM
 * was enabled, and passed with PSRAM off. The pin callbacks it dispatches are
 * application code and may legitimately live in flash - that is the caller's
 * choice - but the driver's own dispatch path must not. */
static void IRAM_ATTR s31_gpio_isr(int vector, void *param)
{
    uint32_t lo, hi;

    RT_UNUSED(vector);
    RT_UNUSED(param);

    gpio_ll_get_intr_status(&GPIO, 0, &lo);
    gpio_ll_get_intr_status_high(&GPIO, 0, &hi);

    if (lo == 0 && hi == 0)
    {
        return;
    }

    /* Clear before dispatching: level triggered pins would otherwise re-enter
     * immediately, and an edge that arrives during the callbacks is picked up
     * by the next iteration anyway. */
    gpio_ll_clear_intr_status(&GPIO, lo);
    gpio_ll_clear_intr_status_high(&GPIO, hi);

    while (lo)
    {
        int n = __builtin_ctz(lo);

        lo &= lo - 1;
        if (s_pin_irq[n].attached && s_pin_irq[n].hdr)
        {
            s_pin_irq[n].hdr(s_pin_irq[n].args);
        }
    }

    while (hi)
    {
        int n = 32 + __builtin_ctz(hi);

        hi &= hi - 1;
        if (n < S31_PIN_NUM && s_pin_irq[n].attached && s_pin_irq[n].hdr)
        {
            s_pin_irq[n].hdr(s_pin_irq[n].args);
        }
    }
}

/* Route, install and unmask once. The CPU line stays open afterwards; the
 * per-pin int_ena bits decide which pads can actually raise it.
 *
 * ---------------------------------------------------------------------------
 * The CPU line is ASKED FOR, not hardcoded, and this is the whole bug.
 *
 * The driver used to call rt_hw_intr_route(ETS_GPIO_INTR0_SOURCE, ETS_GPIO_INUM)
 * with ETS_GPIO_INUM == 4, on the assumption that line 4 is "the GPIO line".
 * It is not. Line 4 belongs to whoever the IDF interrupt allocator handed it to,
 * and on this image that is the OS tick:
 *
 *   vSystimerSetup()  (freertos/port_systick.c:86)
 *       esp_intr_alloc(ETS_SYSTIMER_TARGET0_INTR_SOURCE + cpuid, level, ...)
 *       -> allocator picks the first usable line, which is 4
 *
 * `s31intr` proves it: with PSRAM on, line 4 reads back as SYSTIMER_TARGET0 with
 * the tick's handler installed. So s31_gpio_irq_setup() was doing three things
 * at once: re-routing the tick source off its own line, overwriting the tick's
 * handler with s31_gpio_isr, and then unmasking. The tick stops, the scheduler
 * stops, and the interrupt watchdog reports it - which is exactly the hang that
 * looked like a "PSRAM bug" for three rounds.
 *
 * Why it looked PSRAM-specific: without PSRAM the allocator happens to give the
 * tick a different line, so line 4 is genuinely free and the hardcoded number
 * works by luck. With PSRAM enabled the MSPI/PSRAM setup allocates a line first,
 * shifting everything by one, and the luck runs out.
 *
 * The fix is to allocate the line through the same allocator, with the same
 * source, so the two can never collide: esp_intr_alloc() knows which lines are
 * taken. The handler still goes through rtt_trampoline (via
 * rt_hw_interrupt_install) so RT-Thread callers see the usual API - only the
 * *number* comes from the allocator instead of from a #define.
 */
static int           s_gpio_intno = -1;
static intr_handle_t s_gpio_intr_handle;

static rt_err_t s31_gpio_irq_setup(void)
{
    esp_err_t err;

    if (s_gpio_isr_ready)
    {
        return RT_EOK;
    }

    /* Allocate with the source, not with a number: the allocator routes the
     * matrix too, so rt_hw_intr_route() must not be called separately any more
     * (doing both would leave the source routed twice, to two different lines).
     *
     * ESP_INTR_FLAG_IRAM: s31_gpio_isr is IRAM_ATTR, and the flag keeps the
     * dispatcher from being swapped to a flash-resident path. */
    err = esp_intr_alloc(ETS_GPIO_INTR0_SOURCE,
                         ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
                         rtt_trampoline, RT_NULL, &s_gpio_intr_handle);
    if (err != ESP_OK)
    {
        rt_kprintf("gpio: esp_intr_alloc failed 0x%x\n", (unsigned)err);
        return -RT_ERROR;
    }

    s_gpio_intno = esp_intr_get_intno(s_gpio_intr_handle);
    if (s_gpio_intno < 0 || s_gpio_intno >= SOC_CPU_INTR_NUM)
    {
        rt_kprintf("gpio: allocator returned line %d\n", s_gpio_intno);
        return -RT_ERROR;
    }

    /* Point the slot at our dispatcher, and give the line a level. install()
     * also assigns the default level when the line is still at 0, which is the
     * case for a freshly allocated one. */
    rt_hw_interrupt_install(s_gpio_intno, s31_gpio_isr, RT_NULL, "gpio");

    /* Ask the slot, not the return value: install() returns RT_NULL for both "no
     * previous handler" and "vector rejected", so the result cannot tell them apart.
     * A rejected install here would leave the pin driver registered and every
     * rt_pin_attach_irq() silently dead. */
    if (intr_handler_get(s_gpio_intno) == RT_NULL)
    {
        rt_kprintf("gpio: INUM %d did not take the handler\n", s_gpio_intno);
        return -RT_ERROR;
    }

    rt_hw_interrupt_umask(s_gpio_intno);

    s_gpio_isr_ready = RT_TRUE;

    rt_kprintf("gpio: irq on line %d (allocated, source %d)\n",
               s_gpio_intno, (int)ETS_GPIO_INTR0_SOURCE);

    return RT_EOK;
}

/* ----------------------------------------------------------------- s31pinirq -- */

/*
 * Report the GPIO interrupt line the allocator gave us, and whether the pin
 * channel is armed. Read-only on purpose.
 *
 * The earlier version of this command stepped through route/install/umask
 * manually to bisect the PSRAM hang. It served its purpose - it showed the line
 * was already taken - and it is gone now because reproducing the setup by hand
 * means hardcoding a line number again, which is the bug it was meant to find.
 */
static int s31_pinirq_probe(rt_base_t pin)
{
    if (!s31_pin_valid(pin))
    {
        rt_kprintf("s31pinirq: pin %d invalid\n", (int)pin);
        return -1;
    }

    if (!s_gpio_isr_ready)
    {
        rt_kprintf("s31pinirq: not set up yet (no pin has raised an IRQ)\n");
        return 0;
    }

    rt_kprintf("s31pinirq: source %d -> line %d (from esp_intr_alloc)\n",
               (int)ETS_GPIO_INTR0_SOURCE, s_gpio_intno);
    rt_kprintf("  line ie=%u handler=%s entries=%u\n",
               (unsigned)(rt_hw_interrupt_is_masked(s_gpio_intno) ? 0 : 1),
               (intr_handler_get(s_gpio_intno) != RT_NULL) ? "yes" : "NO",
               (unsigned)rt_hw_interrupt_entry_count(s_gpio_intno));
    rt_kprintf("  pin  %d: int_ena=%u status=%u\n",
               (int)pin,
               (unsigned)GPIO.pin[pin].int_ena,

               (unsigned)((pin < 32) ? ((GPIO.intr_0.int_0 >> pin) & 1u)
                                     : ((GPIO.intr_01.int_01 >> (pin - 32)) & 1u)));

    return 0;
}
static int cmd_pinirq(int argc, char **argv)
{
    rt_base_t pin = 10;

    if (argc >= 2)
    {
        pin = (rt_base_t)atoi(argv[1]);
    }

    return s31_pinirq_probe(pin);
}
MSH_CMD_EXPORT_ALIAS(cmd_pinirq, s31pinirq, show the GPIO IRQ line and pin channel state);

static rt_err_t s31_pin_attach_irq(struct rt_device *device, rt_base_t pin,
                                   rt_uint8_t mode,
                                   void (*hdr)(void *args), void *args)
{
    RT_UNUSED(device);

    if (!s31_pin_valid(pin) || mode > PIN_IRQ_MODE_LOW_LEVEL)
    {
        return -RT_EINVAL;
    }

    /* A stale latched bit from an earlier configuration would fire as soon as
     * the pin is enabled again. */
    gpio_ll_intr_disable(&GPIO, pin);
    s31_pin_clear_irq(pin);
    gpio_ll_set_intr_type(&GPIO, pin, s31_intr_type[mode]);

    s_pin_irq[pin].hdr      = hdr;
    s_pin_irq[pin].args     = args;
    s_pin_irq[pin].mode     = mode;
    s_pin_irq[pin].attached = (hdr != RT_NULL) ? RT_TRUE : RT_FALSE;

    return RT_EOK;
}

static rt_err_t s31_pin_detach_irq(struct rt_device *device, rt_base_t pin)
{
    RT_UNUSED(device);

    if (!s31_pin_valid(pin))
    {
        return -RT_EINVAL;
    }

    gpio_ll_intr_disable(&GPIO, pin);
    s31_pin_clear_irq(pin);

    s_pin_irq[pin].hdr      = RT_NULL;
    s_pin_irq[pin].args     = RT_NULL;
    s_pin_irq[pin].attached = RT_FALSE;

    return RT_EOK;
}

static rt_err_t s31_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                   rt_uint8_t enabled)
{
    RT_UNUSED(device);

    if (!s31_pin_valid(pin))
    {
        return -RT_EINVAL;
    }

    if (!enabled)
    {
        gpio_ll_intr_disable(&GPIO, pin);
        return RT_EOK;
    }

    if (s31_gpio_irq_setup() != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* Channel 0 only: see the file header for the four-channel note. */
    gpio_ll_intr_enable_on_core(&GPIO, 0, pin);

    return RT_EOK;
}

/* Accept "12", "P12", "p12" and "GPIO12". Everything else is rejected so a
 * typo cannot silently resolve to pin 0. */
static rt_base_t s31_pin_get(const char *name)
{
    rt_base_t pin = 0;
    int i = 0;

    if (name == RT_NULL || name[0] == '\0')
    {
        return -1;
    }

    if (name[0] == 'P' || name[0] == 'p')
    {
        i = 1;
    }

    if ((name[i]     == 'G' || name[i]     == 'g') &&
        (name[i + 1] == 'P' || name[i + 1] == 'p') &&
        (name[i + 2] == 'I' || name[i + 2] == 'i') &&
        (name[i + 3] == 'O' || name[i + 3] == 'o'))
    {
        i += 4;
    }

    if (name[i] == '\0')
    {
        return -1;
    }

    for (; name[i] != '\0'; i++)
    {
        if (name[i] < '0' || name[i] > '9')
        {
            return -1;
        }

        pin = pin * 10 + (name[i] - '0');
        if (pin >= S31_PIN_NUM)
        {
            return -1;
        }
    }

    return s31_pin_valid(pin) ? pin : -1;
}

/* The IO_MUX glitch filter has a fixed width (driven by the IO_MUX clock
 * divider); only on/off is exposed. A non-zero request therefore turns the
 * filter on, it does not program the requested width. */
static rt_err_t s31_pin_debounce(struct rt_device *device, rt_base_t pin,
                                 rt_uint32_t debounce)
{
    RT_UNUSED(device);

    if (!s31_pin_valid(pin))
    {
        return -RT_EINVAL;
    }

    if (debounce)
    {
        gpio_ll_pin_filter_enable(&GPIO, pin);
    }
    else
    {
        gpio_ll_pin_filter_disable(&GPIO, pin);
    }

    return RT_EOK;
}

static const struct rt_pin_ops s31_pin_ops =
{
    .pin_mode       = s31_pin_mode,
    .pin_write      = s31_pin_write,
    .pin_read       = s31_pin_read,
    .pin_attach_irq = s31_pin_attach_irq,
    .pin_detach_irq = s31_pin_detach_irq,
    .pin_irq_enable = s31_pin_irq_enable,
    .pin_get        = s31_pin_get,
    .pin_debounce   = s31_pin_debounce,
};

/*
 * Register the pin device the RT-Thread way.
 *
 * Upstream BSPs write `INIT_BOARD_EXPORT(rt_hw_pin_init)` and never mention the
 * function again; rt_hw_board_init() -> rt_components_board_init() walks the
 * .rti_fn board band and calls it. This port called it by hand from board.c
 * instead, because with --gc-sections and no reference the archive member would
 * not otherwise be pulled in.
 *
 * Both halves are being restored here: the export replaces the hand call, and the
 * return type becomes rt_err_t, which is what an RT-Thread init function returns.
 * If the device still shows up in `list device` afterwards, the board band reaches
 * this object on its own and every other driver can follow the same pattern.
 */
rt_err_t rt_hw_pin_init(void)
{
    return rt_device_pin_register("pin", &s31_pin_ops, RT_NULL);
}
INIT_BOARD_EXPORT(rt_hw_pin_init);
