#ifndef PORT_I2C_H
#define PORT_I2C_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One HP I2C controller on this chip (SOC_HP_I2C_NUM == 1). The LP controller
 * is a separate register block and is not driven here. */
#define C5_I2C_HP_NUM       1

/* Free on this board: the console is UART0 on IO11/IO12, the PWM self test uses
 * IO3/IO5/IO7, and the in-package flash pads are not in this range. Both are
 * ordinary GPIOs, so an external 4.7 kOhm pull-up is all a real bus needs. */
#define C5_I2C0_SCL_PIN     8
#define C5_I2C0_SDA_PIN     9

rt_err_t rt_hw_i2c_init(void);

/* Register one bus on the given pads. Exposed because an application that needs
 * different pins has to be able to re-route the bus. */
rt_err_t c5_i2c_init_port(rt_uint32_t port, rt_base_t scl, rt_base_t sda,
                          rt_uint32_t freq);

#ifdef __cplusplus
}
#endif

#endif /* PORT_I2C_H */
