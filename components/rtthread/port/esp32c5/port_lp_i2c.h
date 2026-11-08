#ifndef PORT_LP_I2C_H
#define PORT_LP_I2C_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The pads are NOT a choice: i2c_ll.h defines
 *   LP_I2C_SDA_IOMUX_PAD 2
 *   LP_I2C_SCL_IOMUX_PAD 3
 * and i2c_common.c refuses any other pin. SPI MISO and the PWM test were moved
 * off these two pads for this reason (to IO23 and IO24). */
#define C5_LP_I2C_SDA_PIN   2
#define C5_LP_I2C_SCL_PIN   3

#define C5_LP_I2C_NAME      "lp_i2c0"
#define C5_LP_I2C_FREQ      100000u

rt_err_t rt_hw_lp_i2c_init(void);
rt_err_t rt_hw_lp_i2c_register(void);

/* Prove the controller is real without an LP slave: idle levels, then a NACK from
 * a reserved address. */
int c5_lp_i2c_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_LP_I2C_H */
