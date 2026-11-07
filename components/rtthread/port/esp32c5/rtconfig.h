/* SPDX-License-Identifier: Apache-2.0 */
/*
 * ESP32-C5 kernel RT-Thread configuration.
 *
 * The upstream RT-Thread checkout supplies src/ and include/ unmodified, so this
 * component owns the target configuration. Do not add kernel headers to this
 * directory - rtconfig.h must remain the only header, so upstream headers are
 * not shadowed.
 *
 * The C5 is the C6's CLIC sibling rather than the S31's: same single RV32IMAC
 * application core, same CLIC with hardware nesting (soc_caps.h defines both
 * SOC_INT_CLIC_SUPPORTED and SOC_INT_HW_NESTED_SUPPORTED), but a standard CLIC
 * (INTTHRESH_STANDARD, mintthresh CSR) instead of the S31's vendor CSRs, and a
 * 48 MHz crystal (measured: esptool reports "Crystal frequency: 48MHz").
 *
 * Keep common settings aligned with port/esp32c6/rtconfig.h.
 */
#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* ESP32-C5: RV32IMAC, one hart, machine mode, CLIC interrupt controller. */
#define ARCH_RISCV
#define ARCH_RISCV32
#define SOC_ESPRESSIF
#define SOC_ESP32_C5
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
 * that file is compiled on every build because the FreeRTOS shim's objects are
 * built on the same machinery whether or not the shim itself is enabled (see the
 * C5 target_srcs list in CMakeLists.txt). Without this the file fails with
 * implicit declarations of rt_event_init/rt_event_send/rt_event_recv, because
 * rtthread.h only declares the IPC API its rtconfig.h asked for.
 *
 * Unused event objects cost nothing in the image: --gc-sections drops them. The
 * S31 configuration defines this for exactly the same reason. */
#define RT_USING_EVENT

/* Device framework + serial v1.
 *
 * With RT_USING_DEVICE on, finsh switches from rt_hw_console_getchar() to
 * reading a console device (components/finsh/shell.c), which is why main()
 * binds the shell to "uart0". The UART0 driver itself is port/esp32c5/serial.c.
 */
#define RT_USING_DEVICE
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V1
#define RT_SERIAL_RB_BUFSZ              64
/* Pin device (components/drivers/pin/dev_pin.c). port/esp32c5/gpio.c plugs its
 * rt_pin_ops in, which is what makes rt_pin_*(), the msh `pin` command and
 * interrupt attachment available. */
#define RT_USING_PIN

/* PWM device (components/drivers/misc/rt_drv_pwm.c). port/esp32c5/pwm.c drives
 * the LEDC through it, so `pwm0` behaves like a PWM device on any other BSP:
 * rt_pwm_set()/rt_pwm_enable() with a nanosecond interface. */
#define RT_USING_PWM

/* I2C device class (components/drivers/i2c/dev_i2c_core.c). port/esp32c5/i2c.c
 * registers `i2c0`, so rt_i2c_transfer() and the `i2cscan` shell command work
 * as on any other BSP. */
#define RT_USING_I2C

/* ADC device class (components/drivers/adc/adc.c). port/esp32c5/adc.c registers
 * `adc1`, so rt_adc_enable() / rt_adc_read() work. */
#define RT_USING_ADC

/* SPI bus class (components/drivers/spi/dev_spi_core.c). port/esp32c5/spi.c
 * registers `spi0`, so rt_spi_transfer() and any SPI device package work. */
#define RT_USING_SPI

/* Pulse encoder class (components/drivers/misc/pulse_encoder.c).
 * port/esp32c5/pcnt.c registers `pcnt0`, so rt_device_read() returns the count. */
#define RT_USING_PULSE_ENCODER

/* CAN device class: drives can0 through rt_can rather than port_twai.h. */
#define RT_USING_CAN

/* Console and shell. FINSH_USING_SYMTAB needs __fsymtab_start/__fsymtab_end and
 * the .rti_fn* table. Both are emitted by ld.flash.sections, which is shared by
 * every target and places them inside .flash.rodata - see the long comment
 * there for why a standalone section breaks SOC_MMU_DI_VADDR_SHARED chips. */
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE              256
/* INIT_*_EXPORT() expands to nothing without this, which would leave the
 * .rti_fn table empty and stop finsh_system_init() from ever running. */
#define RT_USING_COMPONENTS_INIT
#define RT_USING_FINSH
#define FINSH_USING_MSH
/* Built-in commands (ps / free) live behind this switch in msh.c:60. Without it
 * the shell still works and `help` still answers, it just lists fewer commands -
 * and `free`/`ps` are exactly the two a bring-up image needs. */
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_THREAD_NAME               "tshell"
#define FINSH_THREAD_PRIORITY           20
#define FINSH_THREAD_STACK_SIZE         4096
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES             5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_USING_OPTION_COMPLETION
#define FINSH_CMD_SIZE                  80
#define FINSH_ARG_MAX                   8

/* Interrupt nesting on ESP32-C5 is done in HARDWARE: soc_caps.h defines
 * SOC_INT_CLIC_SUPPORTED and SOC_INT_HW_NESTED_SUPPORTED, so the trap path in
 * vectors_rtthread.S does not touch the interrupt threshold itself - the CLIC
 * raises mintstatus on entry and restores it on mret. It only has to save this
 * level's mcause before returning, because nested interrupts overwrite it.
 *
 * Do not define SOC_INT_HW_NESTED_SUPPORTED here. It is an SoC capability read
 * from soc/soc_caps.h by the assembly and by C code that branches on it.
 */

/* The C5 has no FPU: its arch string is rv32imac_zicsr_zifencei_zaamo_zalrsc
 * (components/soc/project_include.cmake), so __riscv_flen is undefined and
 * vectors_rtthread.S builds no FP frame. Defining ARCH_RISCV_FPU here would
 * only add 128 bytes to every trap frame for registers that do not exist. */

/* Still undefined on purpose, not by accident:
 *
 *   RT_USING_SOCKET / RT_USING_LIBC  - the lwIP port and newlib are not wired in
 *   RT_USING_DFS                      - no filesystem, so no /dev and no ymodem
 *   RT_USING_SMART                    - that is the microkernel variant
 *   RT_USING_SMP                      - one core; startup.c #errors if IDF runs
 *                                       more cores than RT_CPUS_NR
 */
/* kservice.c compiles the generic backtrace walker unconditionally. */
#define RT_BACKTRACE_LEVEL_MAX_NR       8

#endif /* RT_CONFIG_H__ */



