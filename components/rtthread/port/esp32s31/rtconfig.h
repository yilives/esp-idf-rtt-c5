/* SPDX-License-Identifier: Apache-2.0 */
/*
 * ESP32-S31 kernel-only RT-Thread configuration.
 *
 * The upstream RT-Thread checkout supplies src/ and include/ unmodified, so this
 * component owns the target configuration. Do not add kernel headers to this
 * directory - rtconfig.h must remain the only header, so upstream headers are
 * not shadowed.
 *
 * Keep common settings aligned with port/esp32c3/rtconfig.h and
 * port/esp32c6/rtconfig.h.
 */
#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* ESP32-S31: RV32 with a 128-bit SIMD datapath on one core, two harts,
 * machine mode, CLIC interrupt controller. */
#define ARCH_RISCV
#define ARCH_RISCV32
#define SOC_ESPRESSIF
#define SOC_ESP32_S31

/* P1/P2 run the kernel on the PRO core only. IDF keeps the APP core in reset
 * because CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE is set in sdkconfig.defaults.
 * Raising this to 2 is part of the SMP work (see plan §P3), not of P1. */
#define RT_CPUS_NR                      1

#define RT_NAME_MAX                     12
/* RV32 psABI requires a 16-byte aligned stack. */
#define RT_ALIGN_SIZE                   16
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX          32
/* Must stay equal to CONFIG_FREERTOS_HZ: the tick comes from the IDF SYSTIMER
 * alarm and xPortSysTickHandler() calls rt_tick_increase() once per alarm. */
#define RT_TICK_PER_SECOND              1000
#define IDLE_THREAD_STACK_SIZE          1024

/* Real system heap: kservice.c -> rt_smem_init() in mem.c. The memory region
 * itself comes from IDF_RTTHREAD_HEAP_SIZE (__heap_start__/__heap_end__). */
#define RT_USING_HEAP
#define RT_USING_SMALL_MEM
#define RT_USING_SMALL_MEM_AS_HEAP

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
/* Event groups. port/freertos/ft_event.c maps xEventGroup* onto rt_event, and
 * esp_wifi/esp_event/lwip all use event groups, so this is not optional. */
#define RT_USING_EVENT

/* Device framework + serial v1 + pin.
 *
 * components/drivers/core/device.c is part of this component's kernel source
 * list, components/drivers/serial/dev_serial.c supplies
 * rt_hw_serial_register(), and port/esp32s31/gpio.c registers the "pin"
 * device. Only the v1 serial API is used: the tty/posix half needs
 * RT_USING_SMART.
 *
 * With RT_USING_DEVICE on, finsh switches from rt_hw_console_getchar() to
 * reading a console device (components/finsh/shell.c), which is why main()
 * binds the shell to "uart0".
 */
#define RT_USING_DEVICE
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V1
#define RT_SERIAL_RB_BUFSZ              64
#define RT_USING_PIN

/* Console and shell.
 *
 * FINSH_USING_SYMTAB needs __fsymtab_start/__fsymtab_end and the .rti_fn*
 * table; both are emitted by esp32s31/sections.ld.in under
 * CONFIG_IDF_RTOS_RTTHREAD.
 */
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE              256
/* INIT_*_EXPORT() expands to nothing without this, which would leave the
 * .rti_fn table empty and stop finsh_system_init() from ever running. */
#define RT_USING_COMPONENTS_INIT
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_THREAD_NAME               "tshell"
#define FINSH_THREAD_PRIORITY           20
#define FINSH_THREAD_STACK_SIZE         4096
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES             5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_CMD_SIZE                  80
#define FINSH_ARG_MAX                   8

/* Interrupt nesting on ESP32-S31 is done in HARDWARE: soc_caps.h defines
 * SOC_INT_CLIC_SUPPORTED and SOC_INT_HW_NESTED_SUPPORTED, so the trap path in
 * vectors_rtthread.S does not touch the interrupt threshold itself - the CLIC
 * raises mintstatus on entry and restores it on mret. It only has to save this
 * level's mcause before returning, because nested interrupts overwrite it.
 *
 * Do not define SOC_INT_HW_NESTED_SUPPORTED here. It is an SoC capability read
 * from soc/soc_caps.h by the assembly and by C code that branches on it.
 *
 * No networking, filesystem, Finsh, devices or console at this stage: those
 * RT_USING_* macros stay undefined rather than being defined as 0.
 */
/* kservice.c compiles the generic backtrace walker unconditionally. */
#define RT_BACKTRACE_LEVEL_MAX_NR       8

#endif /* RT_CONFIG_H__ */
