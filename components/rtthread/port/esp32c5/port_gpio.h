/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __PORT_GPIO_H__
#define __PORT_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ESP32-C5 pin device.
 *
 * Registers "pin" through rt_device_pin_register(), so rt_pin_mode(),
 * rt_pin_write(), rt_pin_read(), rt_pin_attach_irq() and the msh `pin` command
 * all work. Called from the BOARD band of the RT-Thread init table, i.e. from
 * rt_hw_board_init() before the scheduler starts.
 *
 * The CMakeLists.txt carries `-u rt_hw_pin_init` because an INIT_BOARD_EXPORT()
 * entry is a data-section reference --gc-sections cannot follow.
 */
rt_err_t rt_hw_pin_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_GPIO_H__ */
