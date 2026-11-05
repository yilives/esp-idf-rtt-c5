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

/* Both cores are configured. What actually runs on CPU1 today is its own idle
 * thread: rt_hw_secondary_cpu_up() starts the core, it takes and releases the
 * scheduler lock (proving the ticket lock works across harts), and then parks in
 * wfi. Taking scheduler work on CPU1 is behind RT_USING_SMP_SLAVE_LOCK and is not
 * currently enabled - measured, it races against CPU0's ready-table edits (see the
 * porting log, W56/W59). The kernel still initialises both per-CPU structures,
 * which is what `s31smp` reports. */
/* The RISC-V port provides rt_hw_atomic_* (atomic_riscv.c). Selecting
 * them keeps every rt_atomic_* call lock-free; the software fallback
 * disables interrupts, which in an SMP build means taking the global
 * spinlock. See the note in CMakeLists.txt. */
#define RT_USING_HW_ATOMIC
#define RT_USING_SMP
#define RT_CPUS_NR                      2
#define RT_USING_SMP_SLAVE
#define RT_USING_SMP_SLAVE_LOCK

/* Let CPU1 run the scheduler instead of parking in wfi.
 *
 * Two things had to exist first and now do: CPU1's own tick
 * (port/esp32s31/smp_tick.c) and the IPI receive side (smp_ipi.c). Without
 * the tick, rt_sched_tick_increase() never runs on this core and its idle
 * thread can never be preempted, so scheduling there would be a no-op.
 *
 * Undefine this to return to the parked-core behaviour that was verified
 * for 20+ rounds, without losing any other change. */
#define RT_SMP_CPU1_SCHEDULES
/*
 * Known: with this on, s31wifi resets the board (CPU_LOCKUP_RESET, or SW_CPU_RESET
 * after the shim tasks were bound to CPU0). RT-Thread threads are fine - s31test is
 * 36/36 and s31smp proves a bound thread runs on CPU1 - but the FreeRTOS shim plus
 * esp_wifi does not survive CPU1 scheduling yet.
 *
 * Binding every shim task to CPU0 (port/freertos/ft_task.c) was the first hypothesis and
 * did not fix it, so the interaction is deeper than task placement. Undefine this to get
 * WiFi back; the cost is CPU1 parking again.
 */

/* Experiment switch: skip CPU1's tick, to isolate what stops its spin loop. */
/* #define RT_SMP_CPU1_NO_TICK */

/* Deliberate fault on CPU1, to test its trap path in isolation. Off except during that
 * one experiment; see smp_slave_sched.c. */
/* #define RT_SMP_CPU1_FAULT_TEST */
/*
 * Off again, with the diagnosis written down.
 *
 * Turning it on gets CPU1 into the scheduler - `s31smp` then reports cpu1 cur_thread ==
 * its idle thread, which a parked core never shows. But:
 *
 *   - a thread created and bound to CPU1 never runs
 *     (`cpu1probe ran on cpu 4294967295, 0 loops in 300 ms`), so CPU1 is not dispatching;
 *   - the board then settles into `rst:0x10 CHIP_LP_WDT_RESET`.
 *
 * Two bugs were found and fixed on the way, and both were real:
 *
 *   1. CPU1 started scheduling before CPU0 had a system up (empty ready table ->
 *      RT_ASSERT(to_thread) -> `rst:0x8 HP_SYS_HP_WDT1_RESET`). Fixed with a go flag set
 *      from INIT_APP_EXPORT, i.e. from main_thread_entry.
 *   2. CPU1 had no tick. rt_tick_increase() keeps this core's tick and expires its time
 *      slice, and rt_timer_check() is CPU0 only by design, so without an interrupt on
 *      CPU1 nothing there can be preempted. `smp_tick.c` now starts TIMG channel 3 from
 *      CPU1 for this (esp_intr_alloc routes a line to the calling core).
 *
 * What is still unexplained is why CPU1 does not dispatch with both in place, and the
 * last measurement narrowed it to one thing rather than a family of them. With
 * interrupts enabled on CPU1 and its TIMG tick configured:
 *
 *     smp: after go, cpu1 alive=0x5a5a5a5a ticks=0 cur=0x2f01fc34 idle=0x2f01fc34
 *
 * That single line says:
 *
 *   - CPU1 ran our entry code (alive is set), so the release works;
 *   - CPU1 reached rt_system_scheduler_start() and switched to its idle thread
 *     (cur == idle), so the scheduler path works;
 *   - CPU1 took ZERO tick interrupts in 300 ms at 1 kHz, so the interrupt never reaches
 *     it. This is not the scheduler declining to switch - it is the core never being
 *     interrupted at all.
 *
 * Also note the difference from IDF, which is where the next attempt should look: IDF
 * runs `esprv_int_set_threshold()` and a global enable at the top of
 * xPortStartScheduler() on *every* core (`components/freertos/FreeRTOS-Kernel/portable/
 * riscv/port.c`), because `mintthresh` is a per-core CSR. This port sets the threshold
 * once, from rt_hw_board_init(), on CPU0. `rt_hw_secondary_cpu_irq_enable()` now does
 * the same two writes on CPU1 and it did not change the result, so the remaining
 * candidates are the interrupt matrix for that source, or `mtvt`/`mtvec` on CPU1
 * (smp_slave.c sets both, but that predates this measurement and should be re-checked
 * with a trap counter rather than assumed).
 *
 * One more attempt was made and it did NOT change the result: enabling CPU1's own CLIC
 * line from CPU1 (`esp_intr_enable()` on the handle s31_timer_start stored) plus
 * mstatus.MIE there. `ticks` stayed 0.
 *
 * That rules out the "allocated and routed but never enabled" explanation, which was the
 * most likely one: esp_intr_alloc() installs and routes but does not enable, and
 * rt_hw_interrupt_umask() writes the CURRENT core's CLIC window
 * (soc/interrupt_reg.h), so it cannot enable CPU1's line from CPU0.
 *
 * Two candidates remain, and they are cheap to tell apart:
 *
 *   1. mtvt/mtvec on CPU1. smp_slave.c sets both from CPU1's entry, but that predates
 *      this measurement and has never been verified with a trap counter. If CPU1 takes
 *      the trap through the wrong table, no handler runs and the interrupt is lost.
 *   2. the interrupt matrix for that source. esp_intr_alloc() programs
 *      esp_rom_route_intr_matrix(cpu, source, intr) with cpu = the CALLING core, so
 *      calling it from CPU1 should route to CPU1 - but that is an assumption about ROM
 *      behaviour on this part, not something measured here.
 *
 * Cheapest next experiment: on CPU1, before scheduling, deliberately fault (read address
 * 0) and see whether the trap handler prints. That tests the vector table with no
 * peripherals or matrix involved, and the answer decides between (1) and (2).
 */

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

/* I2C bus device class (components/drivers/i2c). port/esp32s31/i2c.c registers
 * "i2c0" through it, which is what makes rt_i2c_transfer() and the `i2cscan`
 * command available.
 *
 * RT_USING_I2C_BUSSING used to sit here. Nothing reads it: dev_i2c_core.c gates
 * on RT_USING_DM only, rtdevice.h gates the header on RT_USING_I2C, none of the
 * 466 upstream BSP rtconfig.h files define it, and a whole-tree search of the
 * kernel finds zero references. A switch no code consults is worse than no
 * switch - it reads as "master/slave support is configured" while the only
 * thing that decides is RT_USING_I2C. Removed. */
#define RT_USING_I2C
#define RT_USING_SPI

/* ADC device class (components/drivers/adc). port/esp32s31/adc.c registers
 * "adc1"/"adc2" through it, which is what makes rt_adc_enable() and
 * rt_adc_read() available. */
#define RT_USING_ADC

/* PWM device class (components/drivers/misc/rt_drv_pwm.c). The LEDC driver
 * registers "pwm0" through it, which is what makes rt_pwm_set()/rt_pwm_enable()
 * and msh's `pwm` command available. */
#define RT_USING_PWM

/* Pulse encoder device class (components/drivers/misc/pulse_encoder.c).
 *
 * This port's PCNT driver used to say "RT-Thread has no pulse-counter class" and
 * exposed a private C API instead. The claim was wrong: 5.4.0 ships the class, and
 * pulse_encoder.c even has a !RT_USING_DEVICE_OPS branch, which is this port's
 * configuration.
 *
 * Enabling it lets the PCNT driver register "pcnt0" as a real rt_device: the
 * count is readable with rt_device_read() and PULSE_ENCODER_CMD_* works - the same
 * shape as pwm0 and adc1. */
#define RT_USING_PULSE_ENCODER

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
/* Built-in commands (ps / free, and `bind` on multi-core) live behind this
 * switch in msh.c:60. Without it they are compiled out silently: the shell still
 * works and `help` still answers, it just lists fewer commands - and the two a
 * bring-up image actually needs (`free` to watch the heap, `ps` to see what is
 * running) are exactly the ones that vanish. Nothing else in this port implies
 * it, so it has to be spelled out. */
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_THREAD_NAME               "tshell"
#define FINSH_THREAD_PRIORITY           20
#define FINSH_THREAD_STACK_SIZE         4096
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES             5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
/* Command option matching, needed for any command that dispatches on a sub-word.
 *
 * Without it finsh.h defines
 *
 *     #define MSH_OPT_ID_GET(fun) ((int)(-1UL))
 *
 * so the switch in rt_drv_pwm.c's pwm_list() can never match its own enum, and
 * `pwm probe pwm0` answers "Please using 'pwm probe <device name>' first." even
 * though pwm0 is registered and visible in `list device`. Measured on the board.
 *
 * cmd.c carries a fallback implementation behind #ifndef, which is why `list` keeps
 * working either way - rt_drv_pwm.c does not, so the pwm command is dead without
 * this. 431 of 466 upstream BSPs enable it and the Kconfig default is y, so this is
 * the mainstream setting rather than an addition.
 *
 * dvfs_cmd.c is the only other consumer and is not built here. */
#define FINSH_USING_OPTION_COMPLETION
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
 */

/* Still undefined on purpose, not by accident. Each of these needs a component
 * that this image does not link, so defining it would compile code whose
 * dependencies are absent rather than add a feature:
 *
 *   RT_USING_SOCKET / RT_USING_LIBC  - the lwIP port and newlib are not wired in
 *   RT_USING_DFS                      - no filesystem, so no /dev and no ymodem
 *   RT_USING_SMART                    - that is the microkernel variant
 *
 * Devices, the shell and the console ARE enabled above; the earlier version of this
 * comment claimed otherwise and had gone stale, which is worse than saying nothing:
 * a reader cannot tell whether a missing macro is a decision or an oversight. */
/* kservice.c compiles the generic backtrace walker unconditionally. */
#define RT_BACKTRACE_LEVEL_MAX_NR       8

#endif /* RT_CONFIG_H__ */
