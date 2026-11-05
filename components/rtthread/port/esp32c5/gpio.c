/*
 * ESP32-C5 GPIO driver: rt_pin_ops over the GPIO LL.
 *
 * Derived from port/esp32s31/gpio.c, which is why the shape is the same. The
 * differences that matter on the C5:
 *
 * 1. 29 pads and ONE GPIO interrupt channel (ETS_GPIO_INTR_SOURCE). The S31 has
 *    four channels and a status register split at pin 32; here everything is in
 *    the low word, and gpio_ll_get_intr_status_high() is a no-op that always
 *    reports zero - see the comment on the ISR.
 *
 * 2. esp_driver_gpio/src/gpio.c is FreeRTOS-free but pulls in the interrupt
 *    allocator and the caps layers. The LL is both smaller and directly
 *    auditable, and this driver touches IO_MUX and the GPIO registers only.
 *
 * The pad must be routed to the GPIO peripheral before input/output enable has
 * any effect: IO_MUX.gpio[n].mcu_sel has to be PIN_FUNC_GPIO. That is step one of
 * what IDF's gpio_input_enable()/gpio_output_enable() do.
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

/* The C5 exposes 29 pads (SOC_GPIO_PIN_COUNT); the flash uses the SPIIO pads,
 * which are not in that range. */
#define C5_PIN_NUM             SOC_GPIO_PIN_COUNT

struct c5_pin_irq
{
    void      (*hdr)(void *args);
    void       *args;
    rt_uint8_t  mode;       /* PIN_IRQ_MODE_* */
    rt_bool_t   attached;
};

static struct c5_pin_irq s_pin_irq[C5_PIN_NUM];
static rt_bool_t          s_gpio_isr_ready;

static rt_bool_t c5_pin_valid(rt_base_t pin)
{
    if (pin < 0 || pin >= C5_PIN_NUM)
    {
        return RT_FALSE;
    }

    return ((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) ? RT_TRUE : RT_FALSE;
}

/* Clear the latched interrupt bit of a single pin. All 29 pads live in the low
 * status word on this SoC, so there is no high half to clear. */
static void c5_pin_clear_irq(rt_base_t pin)
{
    gpio_ll_clear_intr_status(&GPIO, 1U << pin);
}

static void c5_pin_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode)
{
    RT_UNUSED(device);

    if (!c5_pin_valid(pin))
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
        /* Keep the receiver on, exactly as the open-drain case above does:
         * rt_pin_read() is documented as "get the level of the pin" with no
         * restriction to input pins, and it is routine to check an output after
         * writing it. The input buffer is off by default on this SoC. */
        gpio_ll_input_enable(&GPIO, pin);
        gpio_ll_output_enable(&GPIO, pin);
        break;

    default:
        break;
    }
}

static void c5_pin_write(struct rt_device *device, rt_base_t pin, rt_uint8_t value)
{
    RT_UNUSED(device);

    if (!c5_pin_valid(pin))
    {
        return;
    }

    gpio_ll_set_level(&GPIO, pin, value ? 1U : 0U);
}

static rt_ssize_t c5_pin_read(struct rt_device *device, rt_base_t pin)
{
    RT_UNUSED(device);

    if (!c5_pin_valid(pin))
    {
        return -RT_EINVAL;
    }

    return gpio_ll_get_level(&GPIO, pin);
}

/* PIN_IRQ_MODE_* is dense and ordered differently from gpio_int_type_t, so map
 * it instead of relying on the numeric values. */
static const gpio_int_type_t c5_intr_type[] =
{
    [PIN_IRQ_MODE_RISING]         = GPIO_INTR_POSEDGE,
    [PIN_IRQ_MODE_FALLING]        = GPIO_INTR_NEGEDGE,
    [PIN_IRQ_MODE_RISING_FALLING] = GPIO_INTR_ANYEDGE,
    [PIN_IRQ_MODE_HIGH_LEVEL]     = GPIO_INTR_HIGH_LEVEL,
    [PIN_IRQ_MODE_LOW_LEVEL]      = GPIO_INTR_LOW_LEVEL,
};

/* One handler for the whole port: the hardware latches per pin and the
 * dispatcher fans out from the status word.
 *
 * IRAM_ATTR, because this runs from an interrupt: with the cache off (any SPI
 * flash write, or an MSPI operation) code fetched from .flash.text cannot be
 * executed. The same annotation has a measured history on the S31 port - its
 * absence turned the first PSRAM bring-up into an interrupt-watchdog timeout
 * with MEPC inside the ISR. The pin callbacks it dispatches are application code
 * and may legitimately live in flash; the driver's own dispatch path must not.
 *
 * The "high" half is deliberately absent: on the C5 gpio_ll_get_intr_status_high()
 * is defined to store zero ("Less than 32 GPIOs"), so calling it and iterating
 * over the result would only add dead code. If a future C5 variant grows past 32
 * pads this has to be revisited, which is why the limitation is written here
 * rather than left to be discovered.
 */
static void IRAM_ATTR c5_gpio_isr(int vector, void *param)
{
    uint32_t lo;

    RT_UNUSED(vector);
    RT_UNUSED(param);

    gpio_ll_get_intr_status(&GPIO, 0, &lo);

    if (lo == 0)
    {
        return;
    }

    /* Clear before dispatching: level triggered pins would otherwise re-enter
     * immediately, and an edge that arrives during the callbacks is picked up
     * by the next iteration anyway. */
    gpio_ll_clear_intr_status(&GPIO, lo);

    while (lo)
    {
        int n = __builtin_ctz(lo);

        lo &= lo - 1;
        if (n < C5_PIN_NUM && s_pin_irq[n].attached && s_pin_irq[n].hdr)
        {
            s_pin_irq[n].hdr(s_pin_irq[n].args);
        }
    }
}

/* Route, install and unmask once. The CPU line stays open afterwards; the
 * per-pin int_ena bits decide which pads can actually raise it.
 *
 * ---------------------------------------------------------------------------
 * The CPU line is ASKED FOR, not hardcoded.
 *
 * On the S31 this exact driver first used ETS_GPIO_INUM == 4 on the assumption
 * that line 4 "is the GPIO line". It is not: line 4 belongs to whoever the IDF
 * interrupt allocator handed it to, and on that image it is the OS tick
 * (vSystimerSetup -> esp_intr_alloc -> first usable line). The result was a
 * silent re-route of the tick's source, its handler overwritten, and a hang that
 * only the interrupt watchdog reported - three rounds of investigation.
 *
 * The C5 has the same allocator and the same tick setup, so the same rule is
 * applied here rather than learned twice: allocate by SOURCE, and take the line
 * number from the allocator. Doing both this and rt_hw_intr_route() would leave
 * the source routed to two different lines, so the route call is gone: the
 * allocator routes the matrix itself.
 */
static int           s_gpio_intno = -1;
static intr_handle_t s_gpio_intr_handle;

static rt_err_t c5_gpio_irq_setup(void)
{
    esp_err_t err;

    if (s_gpio_isr_ready)
    {
        return RT_EOK;
    }

    /* ESP_INTR_FLAG_IRAM: c5_gpio_isr is IRAM_ATTR, and the flag keeps the
     * dispatcher from being swapped to a flash-resident path. */
    err = esp_intr_alloc(ETS_GPIO_INTR_SOURCE,
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
    rt_hw_interrupt_install(s_gpio_intno, c5_gpio_isr, RT_NULL, "gpio");

    /* Ask the slot, not the return value: install() returns RT_NULL for both "no
     * previous handler" and "vector rejected", so the result cannot tell them
     * apart. A rejected install here would leave the pin driver registered and
     * every rt_pin_attach_irq() silently dead. */
    if (intr_handler_get(s_gpio_intno) == RT_NULL)
    {
        rt_kprintf("gpio: INUM %d did not take the handler\n", s_gpio_intno);
        return -RT_ERROR;
    }

    rt_hw_interrupt_umask(s_gpio_intno);

    s_gpio_isr_ready = RT_TRUE;

    rt_kprintf("gpio: irq on line %d (allocated, source %d)\n",
               s_gpio_intno, (int)ETS_GPIO_INTR_SOURCE);

    return RT_EOK;
}

static rt_err_t c5_pin_attach_irq(struct rt_device *device, rt_base_t pin,
                                  rt_uint8_t mode,
                                  void (*hdr)(void *args), void *args)
{
    RT_UNUSED(device);

    if (!c5_pin_valid(pin) || mode > PIN_IRQ_MODE_LOW_LEVEL)
    {
        return -RT_EINVAL;
    }

    /* A stale latched bit from an earlier configuration would fire as soon as
     * the pin is enabled again. */
    gpio_ll_intr_disable(&GPIO, pin);
    c5_pin_clear_irq(pin);
    gpio_ll_set_intr_type(&GPIO, pin, c5_intr_type[mode]);

    s_pin_irq[pin].hdr      = hdr;
    s_pin_irq[pin].args     = args;
    s_pin_irq[pin].mode     = mode;
    s_pin_irq[pin].attached = (hdr != RT_NULL) ? RT_TRUE : RT_FALSE;

    return RT_EOK;
}

static rt_err_t c5_pin_detach_irq(struct rt_device *device, rt_base_t pin)
{
    RT_UNUSED(device);

    if (!c5_pin_valid(pin))
    {
        return -RT_EINVAL;
    }

    gpio_ll_intr_disable(&GPIO, pin);
    c5_pin_clear_irq(pin);

    s_pin_irq[pin].hdr      = RT_NULL;
    s_pin_irq[pin].args     = RT_NULL;
    s_pin_irq[pin].attached = RT_FALSE;

    return RT_EOK;
}

static rt_err_t c5_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                  rt_uint8_t enabled)
{
    RT_UNUSED(device);

    if (!c5_pin_valid(pin))
    {
        return -RT_EINVAL;
    }

    if (!enabled)
    {
        gpio_ll_intr_disable(&GPIO, pin);
        return RT_EOK;
    }

    if (c5_gpio_irq_setup() != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* Channel 0 is the only channel: gpio_ll_intr_enable_on_core() asserts
     * core_id == 0 on this single-core SoC. */
    gpio_ll_intr_enable_on_core(&GPIO, 0, pin);

    return RT_EOK;
}

/* Accept "12", "P12", "p12" and "GPIO12". Everything else is rejected so a
 * typo cannot silently resolve to pin 0. */
static rt_base_t c5_pin_get(const char *name)
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
        if (pin >= C5_PIN_NUM)
        {
            return -1;
        }
    }

    return c5_pin_valid(pin) ? pin : -1;
}

/* The IO_MUX glitch filter has a fixed width (driven by the IO_MUX clock
 * divider); only on/off is exposed. A non-zero request therefore turns the
 * filter on, it does not program the requested width. */
static rt_err_t c5_pin_debounce(struct rt_device *device, rt_base_t pin,
                                rt_uint32_t debounce)
{
    RT_UNUSED(device);

    if (!c5_pin_valid(pin))
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

static const struct rt_pin_ops c5_pin_ops =
{
    .pin_mode       = c5_pin_mode,
    .pin_write      = c5_pin_write,
    .pin_read       = c5_pin_read,
    .pin_attach_irq = c5_pin_attach_irq,
    .pin_detach_irq = c5_pin_detach_irq,
    .pin_irq_enable = c5_pin_irq_enable,
    .pin_get        = c5_pin_get,
    .pin_debounce   = c5_pin_debounce,
};

/*
 * Register the pin device the RT-Thread way: INIT_BOARD_EXPORT() and nothing
 * else. rt_hw_board_init() -> rt_components_board_init() walks the .rti_fn board
 * band and calls it.
 *
 * The CMakeLists.txt `-u rt_hw_pin_init` is what keeps this object in the image:
 * a .rti_fn table entry is data, not a reference, so --gc-sections would
 * otherwise drop the whole member - measured on the S3 port, where `list device`
 * lost `pin` while the file had compiled fine.
 */
rt_err_t rt_hw_pin_init(void)
{
    return rt_device_pin_register("pin", &c5_pin_ops, RT_NULL);
}
INIT_BOARD_EXPORT(rt_hw_pin_init);
