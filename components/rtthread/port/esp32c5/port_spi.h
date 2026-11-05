#ifndef PORT_SPI_H
#define PORT_SPI_H

#include <rtthread.h>
#include "soc/spi_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One GPSPI controller on this chip: SPI2 (SOC_SPI_PERIPH_NUM counts MSPI too,
 * and SPI_LL_GET_HW returns NULL for anything that is not SPI2_HOST). */
#define C5_SPI_NUM          1
#define C5_SPI_DEV_NAME     "spi0"

/* The C5's own IOMUX defaults for GPSPI2 (esp_hal_gpspi/esp32c5/spi_pins.h).
 * Free on this board: console is IO11/IO12, PWM IO3/IO5/IO7, I2C IO8/IO9,
 * in-package flash IO15-IO22. */
#define C5_SPI0_SCLK_PIN    SPI2_IOMUX_PIN_NUM_CLK      /* 6 */
#define C5_SPI0_MOSI_PIN    SPI2_IOMUX_PIN_NUM_MOSI     /* 7 */
#define C5_SPI0_MISO_PIN    SPI2_IOMUX_PIN_NUM_MISO     /* 2 */
#define C5_SPI0_CS_PIN      SPI2_IOMUX_PIN_NUM_CS       /* 10 */

/* Jumper MOSI to MISO to make this prove the data path, not just the registers. */
#define C5_SPI0_LOOP_MOSI   C5_SPI0_MOSI_PIN
#define C5_SPI0_LOOP_MISO   C5_SPI0_MISO_PIN

struct c5_spi_pins
{
    rt_base_t sclk;
    rt_base_t mosi;
    rt_base_t miso;
    rt_base_t cs;       /* -1 for "no hardware CS" */
};

rt_err_t rt_hw_spi_init(void);

rt_err_t c5_spi_init_port(rt_uint32_t port, const struct c5_spi_pins *pins,
                          rt_uint32_t freq);

/* Transfer an 8-byte pattern; returns 0 on a clean run (see the .c for what
 * "all zeros" means). */
int c5_spi_selftest(rt_uint32_t port, rt_bool_t inside);

#ifdef __cplusplus
}
#endif

#endif /* PORT_SPI_H */


