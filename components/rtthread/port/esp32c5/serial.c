/*
 * ESP32-C5 UART0 driver, RT-Thread serial v1 backend over the UART LL.
 *
 * Derived from port/esp32s31/serial.c. The LL API is the same shape on both
 * chips (it is generated from the same CSV description), so what changes is the
 * SoC name and the clock note in configure().
 *
 * Why the LL layer and not esp_driver_uart: uart.c in that component calls
 * xSemaphoreTake/xQueueSendFromISR/xTaskGetTickCount. Those symbols resolve,
 * because the freertos component is still built, but they block on a scheduler
 * and a tick that never run - the image links and then hangs. Everything below
 * touches registers only.
 *
 * UART0 is left exactly as the ROM console configured it: baud rate, frame
 * format and pin mux are all inherited. What is NOT inherited is the interrupt
 * matrix route - cpu_start.c routes every source to ETS_INVALID_INUM (0) and the
 * registers reset to 0 - so init explicitly routes ETS_UART0_INTR_SOURCE onto
 * ETS_UART0_INUM. Without that the handler installs fine and never runs.
 */

#include <rthw.h>
#include "esp_rom_sys.h"
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/uart_struct.h"
#include "soc/uart_reg.h"
#include "soc/interrupts.h"
#include "hal/uart_ll.h"
#include "esp_attr.h"
#include "esp_rom_serial_output.h"

#include "riscv/interrupt.h"
#include "port_intr.h"
#include "port_serial.h"

/* Rate the ROM console was configured with. sdkconfig owns it; the fallback
 * only covers a build where the console is disabled. */
#ifndef CONFIG_ESP_CONSOLE_UART_BAUDRATE
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE 115200
#endif
#define C5_UART0_BAUD          CONFIG_ESP_CONSOLE_UART_BAUDRATE

/* RX timeout in bit times: raise an interrupt for a short burst instead of
 * waiting for the FIFO to fill, which is what makes an interactive shell work. */
#define UART0_RX_TOUT_THRESH    10

#define UART0_RX_INTR_MASK      (UART_RXFIFO_FULL_INT_RAW | UART_RXFIFO_TOUT_INT_RAW)

static struct rt_serial_device serial0;

static rt_err_t c5_uart_configure(struct rt_serial_device *serial,
                                  struct serial_configure  *cfg)
{
    RT_UNUSED(serial);

    /* Inherit the ROM console configuration. The only thing this driver owns
     * is the RX timeout, so a single typed character interrupts. */
    uart_ll_set_rx_tout(&UART0, UART0_RX_TOUT_THRESH);

    if (cfg->baud_rate != C5_UART0_BAUD)
    {
        /* Honest failure instead of a silent no-op. Changing the rate needs the
         * UART source clock programmed by the console setup, and re-deriving it
         * here would silently produce the wrong divider if the assumption about
         * the source is off - the one failure mode that leaves the board with no
         * console at all. */
        rt_kprintf("uart0: %d baud not supported, console stays at %d\n",
                   cfg->baud_rate, C5_UART0_BAUD);
        return -RT_EINVAL;
    }

    return RT_EOK;
}

static rt_err_t c5_uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    RT_UNUSED(serial);
    RT_UNUSED(arg);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        uart_ll_disable_intr_mask(&UART0, UART0_RX_INTR_MASK);
        rt_hw_interrupt_mask(ETS_UART0_INUM);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        uart_ll_ena_intr_mask(&UART0, UART0_RX_INTR_MASK);
        rt_hw_interrupt_umask(ETS_UART0_INUM);
        break;

    default:
        break;
    }

    return RT_EOK;
}

static int c5_uart_putc(struct rt_serial_device *serial, char c)
{
    uint8_t byte = (uint8_t)c;

    RT_UNUSED(serial);

    /* Poll instead of enabling TX interrupts: the shell writes a few bytes at
     * a time and this keeps the driver free of a TX ring buffer. */
    while (uart_ll_get_txfifo_len(&UART0) == 0)
    {
    }
    uart_ll_write_txfifo(&UART0, &byte, 1);

    return 1;
}

static int c5_uart_getc(struct rt_serial_device *serial)
{
    uint8_t byte;

    RT_UNUSED(serial);

    if (uart_ll_get_rxfifo_len(&UART0) == 0)
    {
        return -1;
    }

    uart_ll_read_rxfifo(&UART0, &byte, 1);
    return (int)byte;
}

static const struct rt_uart_ops c5_uart_ops =
{
    .configure    = c5_uart_configure,
    .control      = c5_uart_control,
    .putc         = c5_uart_putc,
    .getc         = c5_uart_getc,
    .dma_transmit = RT_NULL,
};

/* Runs through vectors_rtthread.S: the CLIC slot for ETS_UART0_INUM (5) is mtvt
 * slot 21, which points at _interrupt_handler. */
static void IRAM_ATTR c5_uart_isr(int vector, void *param)
{
    /* The MASKED view, not int_raw.
     *
     * Reading int_raw acknowledges every latched bit, including the error flags -
     * RXFIFO overflow, parity, frame error. They were then cleared by the write
     * below and never reported, so an overrun (a pasted command line, or the
     * shell preempted for a few milliseconds while 64 bytes of input arrive)
     * silently vanished. int_st is raw AND the enable mask, so only the
     * interrupts this driver asked for are cleared and handled.
     */
    rt_uint32_t status = uart_ll_get_intsts_mask(&UART0);
    static rt_uint32_t overruns;

    RT_UNUSED(vector);
    RT_UNUSED(param);

    uart_ll_clr_intsts_mask(&UART0, status);

    /* dev_serial.c's v1 RX path ignores the error bits in `event` (it switches
     * on `event & 0xff` and only acts on RX_IND), so passing them up would be
     * lost anyway. Report the loss directly and rate-limit it: a bring-up image
     * wants "input was dropped" printed once, not a silent truncation.
     *
     * esp_rom_printf, not rt_kprintf: this function lives in IRAM so it can run
     * while the flash cache is off, and rt_kprintf is in IROM. */
    if (status & UART_RXFIFO_OVF_INT_RAW)
    {
        if (overruns++ == 0)
        {
            esp_rom_printf("uart0: RX FIFO overrun (input dropped; the 64-byte "
                           "ring cannot keep up)\n");
        }
    }

    if (status & UART0_RX_INTR_MASK)
    {
        rt_hw_serial_isr(&serial0, RT_SERIAL_EVENT_RX_IND);
    }
}

rt_err_t rt_hw_uart0_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    /* Report the rate the console is really running at, so a mismatch with
     * RT_SERIAL_CONFIG_DEFAULT's 115200 shows up at open() rather than as
     * garbled output. */
    config.baud_rate = C5_UART0_BAUD;

    serial0.ops    = &c5_uart_ops;
    serial0.config = config;

    if (rt_hw_serial_register(&serial0, "uart0",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              RT_NULL) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* Step 1 of four: put UART0 on its own CPU line. Step 2: install the
     * handler. Steps 3 and 4 (unmask the line, enable the peripheral interrupt)
     * happen when the device is opened, see control(). */
    if (rt_hw_intr_route(ETS_UART0_INTR_SOURCE, ETS_UART0_INUM) != 0)
    {
        return -RT_ERROR;
    }

    /* Confirm the slot took the handler rather than trusting the return value.
     *
     * rt_hw_interrupt_install() returns RT_NULL both for "installed, there was no
     * previous handler" - the normal case here - and for "vector rejected". Testing
     * the result therefore proves nothing; asking the slot does. A driver that failed
     * this way would initialise cleanly, register cleanly, and never receive a byte.
     */
    rt_hw_interrupt_install(ETS_UART0_INUM, c5_uart_isr, RT_NULL, "uart0");

    if (intr_handler_get(ETS_UART0_INUM) == RT_NULL)
    {
        rt_kprintf("uart0: INUM %d did not take the handler\n", ETS_UART0_INUM);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/*
 * Console output, in the driver layer where RT-Thread expects it.
 *
 * Upstream BSPs define rt_hw_console_output() in board.c or drv_console.c, not in
 * applications/, and the application is supposed to be portable across boards.
 *
 * Uses the ROM's UART0 writer on purpose: it needs no driver state, so a fault in
 * the serial driver cannot take the console down with it, and rt_kprintf stays
 * usable from the panic path. The port deliberately never calls
 * rt_console_set_device(), so this function is what rt_kprintf actually uses -
 * see _kputs() in kservice.c, which falls back here when no console device is
 * set.
 */
void rt_hw_console_output(const char *str)
{
    while (*str)
    {
        if (*str == '\n')
        {
            esp_rom_output_putc('\r');
        }
        esp_rom_output_putc(*str++);
    }
}

/*
 * Register uart0 the RT-Thread way.
 *
 * The `-u rt_hw_uart0_init` in CMakeLists.txt keeps the archive member alive:
 * INIT_BOARD_EXPORT() writes a data-section (the .rti_fn table) entry, and a data
 * entry is not a reference the linker can follow, so --gc-sections drops the
 * whole object without it.
 */
INIT_BOARD_EXPORT(rt_hw_uart0_init);
