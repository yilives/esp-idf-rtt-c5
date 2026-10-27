/*
 * ESP32-S31 UART0 driver, RT-Thread serial v1 backend over the UART LL.
 *
 * Why the LL layer and not esp_driver_uart: uart.c in that component calls
 * xSemaphoreTake/xQueueSendFromISR/xTaskGetTickCount (risk R10 in the porting
 * plan). Those symbols resolve, because the freertos component is still built,
 * but they block on a scheduler and a tick that never run -- the image links
 * and then hangs. Everything below touches registers only.
 *
 * UART0 is left exactly as the ROM console configured it: baud rate, frame
 * format and pin mux are all inherited. What is NOT inherited is the interrupt
 * matrix route - every INTERRUPT_CORE0_*_MAP register resets to 0 - so
 * init explicitly routes ETS_UART0_INTR_SOURCE onto ETS_UART0_INUM. Without
 * that the handler installs fine and then never runs. Re-programming the baud
 * rate needs the clock-tree work (uart_ll_set_baudrate wants the APB frequency,
 * the pins need the GPIO matrix) which is a later step; until then configure()
 * only records the request.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/uart_struct.h"
#include "soc/uart_reg.h"
#include "soc/interrupts.h"
#include "hal/uart_ll.h"

#include "port_intr.h"
#include "port_serial.h"

/* Rate the ROM console was configured with. sdkconfig owns it; the fallback
 * only covers a build where the console is disabled. */
#ifndef CONFIG_ESP_CONSOLE_UART_BAUDRATE
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE 115200
#endif
#define S31_UART0_BAUD          CONFIG_ESP_CONSOLE_UART_BAUDRATE

/* RX timeout in bit times: raise an interrupt for a short burst instead of
 * waiting for the FIFO to fill, which is what makes an interactive shell work. */
#define UART0_RX_TOUT_THRESH    10

#define UART0_RX_INTR_MASK      (UART_RXFIFO_FULL_INT_RAW | UART_RXFIFO_TOUT_INT_RAW)

static struct rt_serial_device serial0;

static rt_err_t s31_uart_configure(struct rt_serial_device *serial,
                                   struct serial_configure  *cfg)
{
    RT_UNUSED(serial);

    /* Inherit the ROM console configuration. The only thing this driver owns
     * is the RX timeout, so a single typed character interrupts. */
    uart_ll_set_rx_tout(&UART0, UART0_RX_TOUT_THRESH);

    if (cfg->baud_rate != S31_UART0_BAUD)
    {
        /* Honest failure instead of a silent no-op. Changing the rate needs the
         * UART source clock frequency, and S31's uart_ll.h has no
         * uart_ll_get_sclk()/uart_ll_set_sclk() pair (only the LP UART does),
         * so the currently selected source cannot be read back and the mux
         * cannot be repointed through the LL. Deriving it from the programmed
         * divider and the assumed console baud would work but fails silently
         * and wrongly if the assumption is off, which is the one failure mode
         * that leaves the board with no console at all. */
        rt_kprintf("uart0: %d baud not supported, console stays at %d\n",
                   cfg->baud_rate, S31_UART0_BAUD);
        return -RT_EINVAL;
    }

    return RT_EOK;
}

static rt_err_t s31_uart_control(struct rt_serial_device *serial, int cmd, void *arg)
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

static int s31_uart_putc(struct rt_serial_device *serial, char c)
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

static int s31_uart_getc(struct rt_serial_device *serial)
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

static const struct rt_uart_ops s31_uart_ops =
{
    .configure    = s31_uart_configure,
    .control      = s31_uart_control,
    .putc         = s31_uart_putc,
    .getc         = s31_uart_getc,
    .dma_transmit = RT_NULL,
};

/* Runs through vectors_rtthread.S: the CLIC slot for ETS_UART0_INUM (5) is
 * mtvt slot 21, which points at _interrupt_handler. */
static void s31_uart_isr(int vector, void *param)
{
    uint32_t status = uart_ll_get_intraw_mask(&UART0);

    RT_UNUSED(vector);
    RT_UNUSED(param);

    /* Clear first: the ring-buffer fill below may take longer than the next
     * character's arrival. */
    uart_ll_clr_intsts_mask(&UART0, status);

    if (status & UART0_RX_INTR_MASK)
    {
        rt_hw_serial_isr(&serial0, RT_SERIAL_EVENT_RX_IND);
    }
}

int rt_hw_uart0_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    /* Report the rate the console is really running at, so a mismatch with
     * RT_SERIAL_CONFIG_DEFAULT's 115200 shows up at open() rather than as
     * garbled output. */
    config.baud_rate = S31_UART0_BAUD;

    serial0.ops    = &s31_uart_ops;
    serial0.config = config;

    if (rt_hw_serial_register(&serial0, "uart0",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              RT_NULL) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* Step 1 of four: put UART0 on its own CPU line. The matrix resets to 0,
     * so UART0 would otherwise raise CPU interrupt 0 while the handler below
     * sits on line 5 - an install that silently never fires.
     * Step 2: install the handler. Steps 3 and 4 (unmask the line, enable the
     * peripheral interrupt) happen when the device is opened, see control(). */
    if (rt_hw_intr_route(ETS_UART0_INTR_SOURCE, ETS_UART0_INUM) != 0)
    {
        return -RT_ERROR;
    }
    rt_hw_interrupt_install(ETS_UART0_INUM, s31_uart_isr, RT_NULL, "uart0");

    return 0;
}
