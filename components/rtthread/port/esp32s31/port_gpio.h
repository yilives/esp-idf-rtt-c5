/*
 * ESP32-S31 pin device.
 */

#ifndef __PORT_GPIO_H__
#define __PORT_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Register the "pin" device.
 *
 * Called from rt_hw_board_init() so rt_pin_*() works before any driver runs.
 * Also keeps the reference that stops the linker from dropping the driver
 * object out of the component archive.
 *
 * @return RT_EOK on success, a negative error code otherwise
 */
int rt_hw_pin_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_GPIO_H__ */
