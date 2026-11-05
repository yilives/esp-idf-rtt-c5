/*
 * I2C master bus on the S31 I2C LL.
 *
 * Two HP I2C ports (the third instance is the LP one, different register block,
 * out of scope). The bus registers itself as an RT-Thread `rt_i2c_bus_device`,
 * so `rt_i2c_transfer()`, the `i2cscan` msh command and every RT-Thread sensor
 * package work unchanged.
 *
 * Pin choice: from port_board.h, which records what the board actually wires.
 * The first version of this file guessed IO4/IO5 - pads that are I2S_LRCK and
 * I2S_DSDIN on this board - so `i2cscan` was probing a bus nothing was on. The
 * board labels its I2C on IO50/IO51 (audio/video header), and that is the only
 * I2C pair it brings out.
 */

#ifndef __PORT_I2C_H__
#define __PORT_I2C_H__

#include <rtthread.h>

#include "port_board.h"

#ifdef __cplusplus
extern "C" {
#endif

#define S31_I2C0_SCL_PIN        BOARD_I2C0_SCL_PIN
#define S31_I2C0_SDA_PIN        BOARD_I2C0_SDA_PIN

/** Bring up i2c0 at 100 kHz. Called from rt_hw_board_init(). */
rt_err_t rt_hw_i2c_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_I2C_H__ */
