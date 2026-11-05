/*
 * SPI master buses on the S31 GPSPI LL.
 *
 * Two general-purpose controllers are exposed as `spi0` (SPI2) and `spi1`
 * (SPI3); SPI0/SPI1 are the cache/flash controllers and are left alone. The
 * buses register as `rt_spi_bus`, so `rt_spi_transfer()`,
 * `rt_spi_send_then_recv()` and the RT-Thread SPI device packages work as-is.
 *
 * Pin choice is in port_board.h - the S31 pads are fully matrix-routable, so
 * there is no "default" SPI pinout to fall back on. Two pairs are reserved:
 *
 *   spi0: the free 1x20P header pins, usable for a breadboarded peripheral
 *   spi1: deliberately NOT the LCD header (IO3..IO7). Those pads are shared
 *         with I2S and are documented separately in port_board.h; a driver that
 *         claimed them at boot would silently break the audio path.
 */

#ifndef __PORT_SPI_H__
#define __PORT_SPI_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

struct s31_spi_pins
{
    rt_base_t sclk;
    rt_base_t mosi;
    rt_base_t miso;
    rt_base_t cs;      /* -1 to leave chip select unwired */
};

/** Bring up spi0 and spi1 at 1 MHz. Called from rt_hw_board_init(). */
rt_err_t rt_hw_spi_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_SPI_H__ */
