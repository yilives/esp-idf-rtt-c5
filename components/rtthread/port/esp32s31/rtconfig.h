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
