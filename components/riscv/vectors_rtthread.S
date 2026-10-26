/*
 * SPDX-FileCopyrightText: 2017-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"

#if CONFIG_IDF_RTOS_RTTHREAD

#include "soc/soc.h"
#include "soc/interrupt_reg.h"
#include "soc/soc_caps.h"
#include "esp_private/vectors_const.h"

/* Register access for the 32-bit RT-Thread context frame. */
#define STORE                   sw
#define LOAD                    lw
#define REGBYTES                4

    .section .exception_vectors.text, "ax"
    .global _interrupt_handler
    .global _tee_interrupt_handler
    .type _interrupt_handler, @function

/* RT-Thread interrupt entry: save registers on the interrupted thread or interrupt
 * stack, and run interrupt services on the system interrupt stack (__stack_end__,
 * provided by the linker). On exit, rt_thread_switch_interrupt_flag determines
 * whether to switch to to_thread. */
_interrupt_handler:
_tee_interrupt_handler:
    /* Save the outermost context on the thread stack and nested contexts on the interrupt stack. */
    addi sp, sp, -32 * REGBYTES             /* Allocate a frame for 32 CPU registers. */
    STORE x1,   1 * REGBYTES(sp)            /* Save ra. */

    STORE x5,   5 * REGBYTES(sp)            /* Save t0 before reading CSRs. */
    csrr  t0, mepc
    STORE t0,   0 * REGBYTES(sp)            /* Save this level's return address before enabling nested interrupts. */
    csrr  t0, mstatus
    STORE t0,   2 * REGBYTES(sp)            /* Save entry mstatus: hardware cleared MIE; MPIE retains the prior interrupt state. */

    /* Save the remaining CPU registers in this level's context frame. */
    STORE x3,   3 * REGBYTES(sp)            /* Save gp explicitly for debugging and context restoration. */
    STORE x4,   4 * REGBYTES(sp)
    STORE x6,   6 * REGBYTES(sp)
    STORE x7,   7 * REGBYTES(sp)
    STORE x8,   8 * REGBYTES(sp)
    STORE x9,   9 * REGBYTES(sp)
    STORE x10, 10 * REGBYTES(sp)
    STORE x11, 11 * REGBYTES(sp)
    STORE x12, 12 * REGBYTES(sp)
    STORE x13, 13 * REGBYTES(sp)
    STORE x14, 14 * REGBYTES(sp)
    STORE x15, 15 * REGBYTES(sp)
    STORE x16, 16 * REGBYTES(sp)
    STORE x17, 17 * REGBYTES(sp)
    STORE x18, 18 * REGBYTES(sp)
    STORE x19, 19 * REGBYTES(sp)
    STORE x20, 20 * REGBYTES(sp)
    STORE x21, 21 * REGBYTES(sp)
    STORE x22, 22 * REGBYTES(sp)
    STORE x23, 23 * REGBYTES(sp)
    STORE x24, 24 * REGBYTES(sp)
    STORE x25, 25 * REGBYTES(sp)
    STORE x26, 26 * REGBYTES(sp)
    STORE x27, 27 * REGBYTES(sp)
    STORE x28, 28 * REGBYTES(sp)
    STORE x29, 29 * REGBYTES(sp)
    STORE x30, 30 * REGBYTES(sp)
    STORE x31, 31 * REGBYTES(sp)

    /* Keep this level's context pointer until it is stored in its own interrupt stack slot. */
    mv    t1, sp

    /* Only the outermost interrupt switches to the system interrupt stack. Nested
     * interrupts already use that stack; reloading __stack_end__ would overwrite
     * the outer frame, so continue growing downward from the current sp. */
    la    t0, rt_interrupt_nest
    lw    t0, 0(t0)
    bnez  t0, _rtthread_nested_entry
    /* Switch to the system interrupt stack for C interrupt service calls. */
    la    sp, __stack_end__
_rtthread_nested_entry:
    /* Reserve 16 bytes per level for C stack alignment. Nested slots grow downward
     * without overwriting outer slots; the outer thread stack pointer does not
     * depend on s0. */
    addi  sp, sp, -16
    STORE t1, 0(sp)

    call  rt_interrupt_enter    /* Enter interrupt context before servicing the interrupt. */

    csrr  s1, mcause
    csrr  s2, mstatus

#if !SOC_INT_HW_NESTED_SUPPORTED
    /* Chips that do not define SOC_INT_HW_NESTED_SUPPORTED, which includes both
     * ESP32-C3 and ESP32-C6 (PLIC), save and raise the interrupt threshold in
     * software to mask interrupts of equal or lower priority. This file sees
     * soc/soc_caps.h, not rtconfig.h, so C6 takes this path as well. */
    /* Save the current interrupt threshold. */
    li    t0, INTERRUPT_CURRENT_CORE_INT_THRESH_REG
    lw    s3, 0(t0)

    /* Raise the threshold based on the interrupt number to mask equal or lower priorities. */
    li    t2, VECTORS_MCAUSE_REASON_MASK
    and   t1, s1, t2      /* t1 = mcause & mask */
    slli  t1, t1, 2       /* t1 = mcause * 4 */
    li    t2, INTERRUPT_PRIO_REG(0)
    add   t1, t2, t1      /* t1 = INTERRUPT_PRIO_REG + 4 * mcause */
    lw    t2, 0(t1)       /* t2 = INTERRUPT_PRIO_REG[mcause] */
    addi  t2, t2, 1       /* t2 = t2 + 1 */
    sw    t2, 0(t0)       /* INTERRUPT_CURRENT_CORE_INT_THRESH_REG = t2 */
    fence
#endif // !SOC_INT_HW_NESTED_SUPPORTED

    li    t0, 0x8
    csrrs t0, mstatus, t0 /* Enable global interrupts to allow nesting. */

    /* Call the C interrupt dispatcher. */
    mv    a0, sp          /* Argument 1: stack pointer. */
    mv    a1, s1          /* Argument 2: interrupt number (mcause). */
    li    t0, VECTORS_MCAUSE_REASON_MASK
    and   a1, a1, t0      /* Mask off the interrupt flag in mcause. */
    jal   _global_interrupt_handler

    li    t0, 0x8
    csrrc t0, mstatus, t0 /* Disable global interrupts before a possible context switch. */

#if !SOC_INT_HW_NESTED_SUPPORTED
    /* Restore the saved interrupt threshold. */
    li    t0, INTERRUPT_CURRENT_CORE_INT_THRESH_REG
    sw    s3, 0(t0)
    fence
#endif // !SOC_INT_HW_NESTED_SUPPORTED

    call  rt_interrupt_leave    /* Leave interrupt context after servicing the interrupt. */

    /* Recover this level's context pointer, releasing its interrupt stack slot. */
    LOAD  sp, 0(sp)

#if SOC_INT_HW_NESTED_SUPPORTED
    /* CLIC chips with hardware nesting (for example C5 or P4, not C3/C6) restore
     * mintstatus from the priority in mcause on mret. Nested interrupts
     * overwrite mcause, so restore this level's entry value saved in s1 before
     * returning. */
    csrw  mcause, s1
#endif // SOC_INT_HW_NESTED_SUPPORTED

    /* Nested exits restore the interrupted handler; only the outermost exit schedules. */
    la    t0, rt_interrupt_nest
    lw    t0, 0(t0)
    bnez  t0, spurious_interrupt

    /* Read the RT-Thread interrupt context switch flag. */
    la    s0, rt_thread_switch_interrupt_flag
    lw    s2, 0(s0)

    /* A zero flag means no scheduling request: return through spurious_interrupt.
     * The stack still belongs to from_thread and no context switch is needed. */
    beqz  s2, spurious_interrupt

    /* A context switch is requested: clear the scheduling flag. */
    sw    zero, 0(s0)

    /* Save the from_thread stack pointer. */
    la    s0, rt_interrupt_from_thread
    LOAD  s1, 0(s0)
    STORE sp, 0(s1)

    /* Load the to_thread stack pointer and switch to its stack. */
    la    s0, rt_interrupt_to_thread
    LOAD  s1, 0(s0)
    LOAD  sp, 0(s1)

spurious_interrupt:
    /* Restore the return address from the selected context for nesting or thread switching. */
    LOAD  a0,  0 * REGBYTES(sp)
    csrw  mepc, a0

    LOAD  x1,   1 * REGBYTES(sp)

    LOAD  t0,   2 * REGBYTES(sp)
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32S31
    /* Common context_gcc.S stores only MPIE for a cooperative switch.
     * Such a frame can also be resumed here by the tick ISR. Keep MPP=M
     * without forcing MPIE: rt_schedule restores its caller's MIE later.
     * ESP32-S31 is machine-mode only and uses the same cooperative frame
     * layout, so it needs the same fix-up.
     */
    li    t1, 0x1800
    or    t0, t0, t1
#endif
    csrw  mstatus, t0                       /* Restore the saved interrupt state, supplying MPP=M for cooperative frames. */

    LOAD  x3,   3 * REGBYTES(sp)            /* restore gp from slot 3 */
    LOAD  x4,   4 * REGBYTES(sp)
    LOAD  x5,   5 * REGBYTES(sp)
    LOAD  x6,   6 * REGBYTES(sp)
    LOAD  x7,   7 * REGBYTES(sp)
    LOAD  x8,   8 * REGBYTES(sp)
    LOAD  x9,   9 * REGBYTES(sp)
    LOAD  x10, 10 * REGBYTES(sp)
    LOAD  x11, 11 * REGBYTES(sp)
    LOAD  x12, 12 * REGBYTES(sp)
    LOAD  x13, 13 * REGBYTES(sp)
    LOAD  x14, 14 * REGBYTES(sp)
    LOAD  x15, 15 * REGBYTES(sp)
    LOAD  x16, 16 * REGBYTES(sp)
    LOAD  x17, 17 * REGBYTES(sp)
    LOAD  x18, 18 * REGBYTES(sp)
    LOAD  x19, 19 * REGBYTES(sp)
    LOAD  x20, 20 * REGBYTES(sp)
    LOAD  x21, 21 * REGBYTES(sp)
    LOAD  x22, 22 * REGBYTES(sp)
    LOAD  x23, 23 * REGBYTES(sp)
    LOAD  x24, 24 * REGBYTES(sp)
    LOAD  x25, 25 * REGBYTES(sp)
    LOAD  x26, 26 * REGBYTES(sp)
    LOAD  x27, 27 * REGBYTES(sp)
    LOAD  x28, 28 * REGBYTES(sp)
    LOAD  x29, 29 * REGBYTES(sp)
    LOAD  x30, 30 * REGBYTES(sp)
    LOAD  x31, 31 * REGBYTES(sp)

    addi  sp, sp, 32 * REGBYTES
    mret
    .size  _interrupt_handler, .-_interrupt_handler

/* No _mtvt_table here on purpose.
 *
 * CLIC targets compile vectors_clic.S unconditionally, and that file already
 * defines _mtvt_table; its free-interrupt slots point at _interrupt_handler,
 * which this file provides. Defining the table here as well would duplicate the
 * symbol. On ESP32-S31 the CPU interrupt lines are 0..31 (SOC_CPU_INTR_NUM=32),
 * the internal ones sit at 25..28, and everything the interrupt matrix routes
 * lands on slots that resolve to _interrupt_handler.
 */

#endif /* CONFIG_IDF_RTOS_RTTHREAD */
