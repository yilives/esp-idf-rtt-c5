#ifndef __PORT_SERIAL_H__
#define __PORT_SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif


/*
 * ESP32-S31 UART0 console driver (RT-Thread serial v1).
 *
 * Registers "uart0" and wires ETS_UART0_INUM through the RT-Thread interrupt
 * bridge. Call once before finsh_set_device("uart0").
 */
int rt_hw_uart0_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_SERIAL_H__ */
