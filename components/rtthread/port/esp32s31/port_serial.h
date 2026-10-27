#ifndef __PORT_SERIAL_H__
#define __PORT_SERIAL_H__

/*
 * ESP32-S31 UART0 console driver (RT-Thread serial v1).
 *
 * Registers "uart0" and wires ETS_UART0_INUM through the RT-Thread interrupt
 * bridge. Call once before finsh_set_device("uart0").
 */
int rt_hw_uart0_init(void);

#endif /* __PORT_SERIAL_H__ */
