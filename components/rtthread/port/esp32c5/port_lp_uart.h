#ifndef PORT_LP_UART_H
#define PORT_LP_UART_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The LP UART's default pads, from `uart_pins.h`: TXD IO5, RXD IO4, LP IO MUX
 * function 0 for both. Unlike LP I2C these are not the only pads the peripheral
 * can reach, but they are the ones the LP IO MUX is documented with. */
#define C5_LP_UART_TXD_PIN   5
#define C5_LP_UART_RXD_PIN   4

#define C5_LP_UART_BAUD      115200u

/* XTAL_D2: the 48 MHz crystal divided by two, which is the LP domain's clock. */
#define C5_LP_UART_SCLK_HZ   24000000u

/* Bring up the LP UART and its pads. Idempotent. */
rt_err_t rt_hw_lp_uart_init(rt_uint32_t baud);

/* Send `len` bytes; with `loopback` also read them back and return the count. */
int rt_hw_lp_uart_xfer(const rt_uint8_t *tx, rt_uint8_t *rx, rt_uint32_t len,
                       rt_bool_t loopback);

/* `c5lpuart [baud]` - internal-loopback round trip plus a pad read-back. */
int c5_lp_uart_selftest(rt_uint32_t baud);

#ifdef __cplusplus
}
#endif

#endif /* PORT_LP_UART_H */
