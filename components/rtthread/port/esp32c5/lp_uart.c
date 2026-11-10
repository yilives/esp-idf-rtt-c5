/*
 * `c5lpuart` - the low-power UART, written the way LP I2C should have been.
 *
 * LP I2C cost thirteen rounds to a pad-configuration bug: a correct `rtc_gpio_*`
 * sequence immediately undone by four leftover raw `rtcio_ll_*` calls below it,
 * which put the pad back on the digital side with the peripheral's input
 * disconnected. This driver is written after that lesson, so:
 *
 *   - the pad setup is IDF's `uart_try_set_iomux_pin()` LP branch
 *     (`uart.c:739-752`): `rtc_gpio_init()`, then `rtc_gpio_iomux_output()` for
 *     TXD or `rtc_gpio_iomux_input()` for RXD - and nothing else touches those
 *     registers afterwards;
 *   - `lp_uart_ll_enable_bus_clock()` comes BEFORE `lp_uart_ll_reset_register()`,
 *     because a reset pulse delivered to an unclocked block is not a reset.
 *
 * ---------------------------------------------------------------------------
 * Why the test uses the hardware loopback.
 *
 * There is no UART peer on this board's LP pads - IO4 and IO5 are the CAN and
 * PCNT test pads, and IO11/IO12 are the console. `uart_ll_set_loop_back()`
 * connects TXD to RXD inside the peripheral, so bytes written come straight back.
 * That exercises the baud generator, the shifter, both FIFOs and the receive
 * path - everything except the pad.
 *
 * **A looped-back transfer proves the peripheral, not the pins**, and the report
 * says so rather than implying a wire was tested. The pads are configured anyway
 * and the RXD level is read back, because a pad setup that silently failed would
 * leave the loopback passing while proving nothing about the pins - which is
 * exactly the shape of the LP I2C failure.
 *
 * ---------------------------------------------------------------------------
 * The quirk that makes LP UART different from the HP instances.
 *
 * `UART_LL_REG_FIELD_BIT_SHIFT(hw)` is 3 for `LP_UART` and 0 for the HP ones
 * (`uart_ll.h:30`): the LP FIFO count fields sit at a 3-bit offset in the same
 * `status` register. Reading them through the struct fields gives a count eight
 * times too large, so every FIFO count below goes through the LL functions,
 * which apply the shift.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"

/* The capability macros on this part are SOC_UART_LP_NUM and
 * SOC_UART_HAS_LP_UART (soc_caps.h:523,530). Guessing SOC_LP_UART_NUM
 * compiled the whole file out without a warning, and the only symptom
 * was an undefined reference at link time. */
#if defined(SOC_UART_LP_NUM) && (SOC_UART_LP_NUM >= 1)

#include "hal/uart_ll.h"
#include "hal/uart_types.h"
#include "soc/uart_pins.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk_tree_common.h"
#include "driver/rtc_io.h"

#include "port_lp_uart.h"

/* The LP instance is not an HP index: `UART_LL_GET_HW()` maps anything other
 * than 0 or 1 to `&LP_UART`, but the id is written out so the intent is visible
 * rather than implied by an else-branch. */
#define C5_LP_UART_HW_ID     SOC_UART_LP_NUM

static uart_dev_t *s_lp_uart;
static rt_bool_t   s_lp_uart_ready;

/* ------------------------------------------------------------------ pads -- */

static void c5_lp_uart_set_pins(void)
{
    /* TXD: IO5, LP IO MUX function 0 (`LP_U0TXD_GPIO_NUM`, uart_pins.h:25).
     *
     * `rtc_gpio_iomux_output()` is the output-side counterpart of the
     * `rtc_gpio_iomux_input()` LP I2C needed - the same section of the same
     * function in IDF (`uart.c:740-745`), differing only in which direction the
     * pad is driven. */
    rtc_gpio_init((gpio_num_t)C5_LP_UART_TXD_PIN);
    rtc_gpio_iomux_output((gpio_num_t)C5_LP_UART_TXD_PIN, LP_U0TXD_MUX_FUNC);

    /* RXD: IO4, function 0, input side. The pulls are disabled afterwards because
     * IDF does that explicitly (`uart.c:746-751`) to undo a workaround applied
     * while the module was enabled; skipping it leaves a pull-up fighting the
     * peer's driver. */
    rtc_gpio_init((gpio_num_t)C5_LP_UART_RXD_PIN);
    rtc_gpio_iomux_input((gpio_num_t)C5_LP_UART_RXD_PIN, LP_U0RXD_MUX_FUNC, 0);
    rtc_gpio_pullup_dis((gpio_num_t)C5_LP_UART_RXD_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)C5_LP_UART_RXD_PIN);
}

/* ------------------------------------------------------------------ init -- */

rt_err_t rt_hw_lp_uart_init(rt_uint32_t baud)
{
    if (baud == 0u || baud > 5000000u)
    {
        baud = C5_LP_UART_BAUD;
    }

    if (s_lp_uart_ready)
    {
        return RT_EOK;
    }

    /* CLOCK FIRST, THEN RESET.
     *
     * `lp_uart_ll_reset_register()` pulses `LPPERI.reset_en.lp_ext_uart_reset_en`,
     * and a reset pulse delivered to a block whose bus clock is off cannot clear
     * its register file. LP I2C had this backwards for thirty rounds; it was one
     * of the thirteen candidates there and is not repeated here. */
    PERIPH_RCC_ATOMIC()
    {
        lp_uart_ll_enable_bus_clock(C5_LP_UART_HW_ID, true);
        lp_uart_ll_reset_register(C5_LP_UART_HW_ID);
        lp_uart_ll_set_source_clk(&LP_UART, LP_UART_SCLK_XTAL_D2);
    }

    /* Turn the source clock on, not just select it - `i2c_common.c:269` does this
     * for I2C, and omitting it was a real difference there. */
    (void)esp_clk_tree_enable_src(SOC_MOD_CLK_XTAL_D2, true);

    s_lp_uart = &LP_UART;

    c5_lp_uart_set_pins();

    uart_ll_set_mode(s_lp_uart, UART_MODE_UART);
    uart_ll_set_data_bit_num(s_lp_uart, UART_DATA_8_BITS);
    uart_ll_set_stop_bits(s_lp_uart, UART_STOP_BITS_1);
    uart_ll_set_parity(s_lp_uart, UART_PARITY_DISABLE);
    uart_ll_set_hw_flow_ctrl(s_lp_uart, UART_HW_FLOWCTRL_DISABLE, 0);

    /* XTAL_D2 is 24 MHz on this board (48 MHz crystal over 2). The divisor comes
     * from that, not from the HP APB clock - the LP peripheral is not on APB. */
    if (!lp_uart_ll_set_baudrate(s_lp_uart, baud, C5_LP_UART_SCLK_HZ))
    {
        rt_kprintf("c5lpuart: baud %u is not achievable from %u Hz\n",
                   (unsigned)baud, (unsigned)C5_LP_UART_SCLK_HZ);
        return -RT_EINVAL;
    }

    uart_ll_set_loop_back(s_lp_uart, false);
    uart_ll_txfifo_rst(s_lp_uart);
    uart_ll_rxfifo_rst(s_lp_uart);
    uart_ll_update(s_lp_uart);

    s_lp_uart_ready = RT_TRUE;

    return RT_EOK;
}

/* ------------------------------------------------------------------ xfer -- */

/* Send `len` bytes and, with `loopback`, read them straight back. Returns the
 * byte count received when looping back, 0 for a write-only transfer, or a
 * negative value on a bad argument. */
int rt_hw_lp_uart_xfer(const rt_uint8_t *tx, rt_uint8_t *rx, rt_uint32_t len,
                       rt_bool_t loopback)
{
    rt_uint32_t sent = 0;
    rt_uint32_t got = 0;
    rt_uint32_t guard = 0;

    if (!s_lp_uart_ready || tx == RT_NULL || len == 0u)
    {
        return -1;
    }

    uart_ll_set_loop_back(s_lp_uart, loopback ? true : false);
    uart_ll_update(s_lp_uart);

    /* INTERLEAVE the write and the read. Filling the whole TX FIFO and only then
     * draining RX loses bytes: the LP receive FIFO is 16 bytes
     * (`SOC_LP_UART_FIFO_LEN`), so a 20-byte message overflows it while the CPU is
     * still pushing the transmit side.
     *
     * Measured exactly that: 16 bytes round-tripped perfectly at 115200 baud, and
     * a 20-byte message failed at 921600 with all 20 bytes wrong. Slower baud
     * only masked it - the CPU finished writing and began reading before the
     * receiver had filled past the FIFO, which is luck, not correctness. The
     * first version of this comment called the 115200 case "passing"; it was
     * passing by accident. */
    while (sent < len || (loopback && got < len))
    {
        rt_uint32_t room;
        rt_uint32_t avail;

        if (sent < len)
        {
            room = uart_ll_get_txfifo_len(s_lp_uart);
            if (room > 0u)
            {
                if (room > (len - sent))
                {
                    room = len - sent;
                }

                uart_ll_write_txfifo(s_lp_uart, tx + sent, room);
                sent += room;
            }
        }

        if (loopback)
        {
            avail = uart_ll_get_rxfifo_len(s_lp_uart);
            if (avail > 0u)
            {
                if (avail > (len - got))
                {
                    avail = len - got;
                }

                uart_ll_read_rxfifo(s_lp_uart, rx + got, avail);
                got += avail;
            }
        }

        if (++guard > 2000000u)
        {
            break;
        }
    }

    /* Only after everything is out AND back does the shifter state matter. */
    for (guard = 0; guard < 200000u; guard++)
    {
        if (uart_ll_is_tx_idle(s_lp_uart))
        {
            break;
        }
    }

    if (!loopback)
    {
        return 0;
    }

    /* Anything that arrived after the interleaved loop finished. */
    while (got < len && guard > 0u)
    {
        rt_uint32_t avail = uart_ll_get_rxfifo_len(s_lp_uart);

        if (avail == 0u)
        {
            guard--;
            continue;
        }

        if (avail > (len - got))
        {
            avail = len - got;
        }

        uart_ll_read_rxfifo(s_lp_uart, rx + got, avail);
        got += avail;
    }

    return (int)got;
}

/* ------------------------------------------------------------------- msh -- */

int c5_lp_uart_selftest(rt_uint32_t baud)
{
    static const rt_uint8_t pattern[16] =
    {
        0x55, 0xAA, 0x00, 0xFF, 0x01, 0x80, 0x7F, 0xFE,
        'L',  'P',  'U',  'A',  'R',  'T',  0x0D, 0x0A
    };
    rt_uint8_t rx[sizeof(pattern)];
    rt_uint32_t i;
    int got;
    int fail = 0;
    rt_err_t err;

    if (baud == 0u || baud > 5000000u)
    {
        baud = C5_LP_UART_BAUD;
    }

    /* Re-program the baud rate on every run, so calling this twice with
     * different rates actually tests different rates. `rt_hw_lp_uart_init()`
     * returns early once ready, so the divisor is set here instead. */
    err = rt_hw_lp_uart_init(baud);

    if (err != RT_EOK)
    {
        rt_kprintf("c5lpuart: FAIL, init returned %d\n", (int)err);
        return -1;
    }

    /* QUIESCE BEFORE CHANGING THE DIVISOR.
     *
     * Measured: `c5lpuart 921600` passes as the first call after boot and fails
     * when it follows `c5lpuart 115200` in the same session - 15 of 16 bytes
     * wrong. Nothing is wrong with either rate; the peripheral was re-programmed
     * while the loopback was still on and the receive FIFO still held the tail of
     * the previous transfer, so the new frame boundaries were decoded against
     * stale samples.
     *
     * Changing a baud divisor is not a register write like any other: the
     * receiver is mid-bit whenever there is traffic, and the two FIFOs hold data
     * clocked at the old rate. */
    uart_ll_set_loop_back(s_lp_uart, false);
    uart_ll_update(s_lp_uart);
    uart_ll_txfifo_rst(s_lp_uart);
    uart_ll_rxfifo_rst(s_lp_uart);
    uart_ll_update(s_lp_uart);

    if (!lp_uart_ll_set_baudrate(s_lp_uart, baud, C5_LP_UART_SCLK_HZ))
    {
        rt_kprintf("c5lpuart: FAIL, %u baud is not achievable from %u Hz\n",
                   (unsigned)baud, (unsigned)C5_LP_UART_SCLK_HZ);
        return -1;
    }
    uart_ll_update(s_lp_uart);

    rt_kprintf("c5lpuart: LP UART, TXD=IO%d RXD=IO%d, %u baud from %u Hz\n",
               (int)C5_LP_UART_TXD_PIN, (int)C5_LP_UART_RXD_PIN,
               (unsigned)baud, (unsigned)C5_LP_UART_SCLK_HZ);

    /* The pad state is asserted separately, and this is the line that matters.
     *
     * The loopback is internal to the peripheral, so a pad configuration that
     * silently failed would still let every byte come back - the test would pass
     * while proving nothing about the pins. That is precisely the LP I2C failure
     * shape: a working controller with disconnected pads. Reading the pad back is
     * the only thing that separates them. */
    rt_kprintf("c5lpuart: RXD pad level %u before any traffic (pulls disabled, so "
               "a floating line settles at 1)\n",
               (unsigned)rtc_gpio_get_level((gpio_num_t)C5_LP_UART_RXD_PIN));

    rt_memset(rx, 0, sizeof(rx));

    got = rt_hw_lp_uart_xfer(pattern, rx, sizeof(pattern), RT_TRUE);

    if (got != (int)sizeof(pattern))
    {
        rt_kprintf("c5lpuart: FAIL, loopback returned %d of %u byte(s)\n",
                   got, (unsigned)sizeof(pattern));
        fail++;
    }
    else
    {
        rt_uint32_t bad = 0;

        for (i = 0; i < sizeof(pattern); i++)
        {
            if (rx[i] != pattern[i])
            {
                if (bad < 4u)
                {
                    rt_kprintf("c5lpuart:   +%u got 0x%02x, sent 0x%02x\n",
                               (unsigned)i, rx[i], pattern[i]);
                }
                bad++;
            }
        }

        if (bad != 0u)
        {
            rt_kprintf("c5lpuart: FAIL, %u of %u byte(s) differ\n",
                       (unsigned)bad, (unsigned)sizeof(pattern));
            fail++;
        }
        else
        {
            rt_kprintf("c5lpuart: %u byte(s) looped back byte-exact\n",
                       (unsigned)sizeof(pattern));
        }
    }

    /* A second transfer of a different length.
     *
     * 20 bytes crosses the 16-byte LP FIFO, which is the case that exposed the
     * interleaving bug: at 921600 baud every byte of this one came back wrong
     * while the 16-byte transfer passed. Trying three lengths, not one, is what
     * makes the FIFO handling rather than the happy path. */
    {
        rt_uint8_t big_tx[40];
        rt_uint8_t big_rx[40];
        const rt_uint32_t sizes[2] = { 20u, 40u };
        rt_uint32_t k;

        for (i = 0; i < 40u; i++)
        {
            big_tx[i] = (rt_uint8_t)(i * 13u + 7u);
        }

        for (k = 0; k < 2u; k++)
        {
            rt_uint32_t n = sizes[k];

            rt_memset(big_rx, 0, sizeof(big_rx));

            got = rt_hw_lp_uart_xfer(big_tx, big_rx, n, RT_TRUE);

            if (got != (int)n || rt_memcmp(big_tx, big_rx, n) != 0)
            {
                rt_kprintf("c5lpuart: FAIL, %u-byte transfer returned %d and %s\n",
                           (unsigned)n, got,
                           (rt_memcmp(big_tx, big_rx, n) == 0) ? "matched"
                                                               : "differed");
                fail++;
            }
            else
            {
                rt_kprintf("c5lpuart: %u byte(s) looped back byte-exact (%u "
                           "FIFO(s))\n", (unsigned)n,
                           (unsigned)(n / SOC_LP_UART_FIFO_LEN));
            }
        }
    }

    uart_ll_set_loop_back(s_lp_uart, false);
    uart_ll_update(s_lp_uart);

    if (fail != 0)
    {
        rt_kprintf("c5lpuart: FAIL, %d check(s) did not hold at %u baud\n",
                   fail, (unsigned)baud);
        return -1;
    }

    rt_kprintf("c5lpuart: PASS at %u baud (the peripheral transmits, receives and "
               "reports idle correctly; the loopback is INTERNAL, so this proves "
               "the LP UART and its clocks - not the pads or a wire)\n",
               (unsigned)baud);
    return 0;
}

#endif /* SOC_UART_LP_NUM >= 1 */
