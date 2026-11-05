/*
 * SPDX-FileCopyrightText: 2017-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"

/* rtconfig.h decides RT_USING_SMP, and the trap path has to know: in an SMP
 * build the three switch globals do not exist and the kernel performs the IRQ
 * switch itself. The RT-Thread component publishes its config directory
 * globally (CMakeLists: idf_build_set_property(INCLUDE_DIRECTORIES ...)), so
 * this include resolves without touching the riscv component's own paths. */
#if CONFIG_IDF_RTOS_RTTHREAD
#include "rtconfig.h"
#endif

#if CONFIG_IDF_RTOS_RTTHREAD

#include "soc/soc.h"
#include "soc/interrupt_reg.h"
#include "soc/soc_caps.h"
#include "esp_private/vectors_const.h"

/* Register access for the 32-bit RT-Thread context frame. */
#define STORE                   sw
#define LOAD                    lw
#define REGBYTES                4

/* The frame must match rt_hw_stack_frame_t (32 integer slots first, then
 * f0..f31 at higher addresses) and port/common/context_gcc.S.
 * __riscv_flen comes from the toolchain -march (the S31 arch string carries
 * f2p2), so this is a property of the build, not an rtconfig.h option. */
#if defined(__riscv_flen) && (__riscv_flen == 32)
#define RT_FPU_FRAME            1
#define FPREG_BYTES             4
#define FPSTORE                 fsw
#define FPLOAD                  flw
#elif defined(__riscv_flen) && (__riscv_flen == 64)
#define RT_FPU_FRAME            1
#define FPREG_BYTES             8
#define FPSTORE                 fsd
#define FPLOAD                  fld
#endif

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
    /* Save the outermost context on the thread stack and nested contexts on the interrupt stack.
     *
     * The frame must have exactly the same shape as the one context_gcc.S builds
     * for a cooperative switch: 32 integer slots followed by 32 FP slots at
     * higher addresses. If the interrupt frame were smaller, the trap exit would
     * return with sp short by the FP area whenever it resumes a cooperatively
     * switched thread - and that corrupts pure-integer threads as well, no
     * floating point needs to be involved.
     */
#ifdef RT_FPU_FRAME
    addi sp, sp, -32 * FPREG_BYTES          /* FP slots live at the higher addresses. */
#endif
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

#ifdef RT_FPU_FRAME
    /* mstatus.FS is non-zero in every context this entry can run on:
     * rt_hw_stack_init_core() builds thread frames with mstatus=0x7880, and
     * rtthread_startup() keeps MIE off until the first thread's mret, so no
     * interrupt can arrive with the FPU disabled. */
    FPSTORE f0,  (32 * REGBYTES +  0 * FPREG_BYTES)(sp)
    FPSTORE f1,  (32 * REGBYTES +  1 * FPREG_BYTES)(sp)
    FPSTORE f2,  (32 * REGBYTES +  2 * FPREG_BYTES)(sp)
    FPSTORE f3,  (32 * REGBYTES +  3 * FPREG_BYTES)(sp)
    FPSTORE f4,  (32 * REGBYTES +  4 * FPREG_BYTES)(sp)
    FPSTORE f5,  (32 * REGBYTES +  5 * FPREG_BYTES)(sp)
    FPSTORE f6,  (32 * REGBYTES +  6 * FPREG_BYTES)(sp)
    FPSTORE f7,  (32 * REGBYTES +  7 * FPREG_BYTES)(sp)
    FPSTORE f8,  (32 * REGBYTES +  8 * FPREG_BYTES)(sp)
    FPSTORE f9,  (32 * REGBYTES +  9 * FPREG_BYTES)(sp)
    FPSTORE f10, (32 * REGBYTES + 10 * FPREG_BYTES)(sp)
    FPSTORE f11, (32 * REGBYTES + 11 * FPREG_BYTES)(sp)
    FPSTORE f12, (32 * REGBYTES + 12 * FPREG_BYTES)(sp)
    FPSTORE f13, (32 * REGBYTES + 13 * FPREG_BYTES)(sp)
    FPSTORE f14, (32 * REGBYTES + 14 * FPREG_BYTES)(sp)
    FPSTORE f15, (32 * REGBYTES + 15 * FPREG_BYTES)(sp)
    FPSTORE f16, (32 * REGBYTES + 16 * FPREG_BYTES)(sp)
    FPSTORE f17, (32 * REGBYTES + 17 * FPREG_BYTES)(sp)
    FPSTORE f18, (32 * REGBYTES + 18 * FPREG_BYTES)(sp)
    FPSTORE f19, (32 * REGBYTES + 19 * FPREG_BYTES)(sp)
    FPSTORE f20, (32 * REGBYTES + 20 * FPREG_BYTES)(sp)
    FPSTORE f21, (32 * REGBYTES + 21 * FPREG_BYTES)(sp)
    FPSTORE f22, (32 * REGBYTES + 22 * FPREG_BYTES)(sp)
    FPSTORE f23, (32 * REGBYTES + 23 * FPREG_BYTES)(sp)
    FPSTORE f24, (32 * REGBYTES + 24 * FPREG_BYTES)(sp)
    FPSTORE f25, (32 * REGBYTES + 25 * FPREG_BYTES)(sp)
    FPSTORE f26, (32 * REGBYTES + 26 * FPREG_BYTES)(sp)
    FPSTORE f27, (32 * REGBYTES + 27 * FPREG_BYTES)(sp)
    FPSTORE f28, (32 * REGBYTES + 28 * FPREG_BYTES)(sp)
    FPSTORE f29, (32 * REGBYTES + 29 * FPREG_BYTES)(sp)
    FPSTORE f30, (32 * REGBYTES + 30 * FPREG_BYTES)(sp)
    FPSTORE f31, (32 * REGBYTES + 31 * FPREG_BYTES)(sp)
#endif

    /* Keep this level's context pointer until it is stored in its own interrupt stack slot. */
    mv    t1, sp

    /* Only the outermost interrupt switches to the system interrupt stack. Nested
     * interrupts already use that stack; reloading __stack_end__ would overwrite
     * the outer frame, so continue growing downward from the current sp. */
    /* This core's interrupt nest counter.
     *
     * The single-core build reads the global directly: one `la` and one load,
     * nothing that can land in flash, and this exact shape has been running
     * since the first bring-up.
     *
     * The SMP build cannot use a global there - rt_interrupt_nest is a macro
     * over rt_cpu_self()->irq_nest (irq.c:68) and has no symbol - so it indexes
     * an IRAM table of per-CPU pointers with mhartid. It cannot use a call
     * either: IDF disables the flash cache while tuning MSPI/PSRAM timing, and a
     * flash-resident callee faults in that window (observed: MEPC 0x40013b1e
     * resolved to rt_enter_critical in IROM).
     *
     * The two builds therefore read this value differently on purpose. */
    la    t2, rt_irq_nest_slot
    csrr  t0, mhartid
    slli  t0, t0, 2
    add   t0, t0, t2
    lw    t0, 0(t0)         /* t0 = slot[n], the address of this core's counter */
    lw    t0, 0(t0)         /* t0 = the counter itself - the branch needs a value,
                             * not a pointer: a pointer is never zero, so a missing
                             * load here makes every interrupt look nested */
    bnez  t0, _rtthread_nested_entry
    /* Switch to this core's interrupt stack for C interrupt service calls.
     *
     * A single literal address was correct while there was one core. With two,
     * both cores would push frames downward from the same place and overwrite
     * each other, so the top comes from a per-CPU table instead - indexed the
     * same way as the nest counter above, and for the same reason (no call: the
     * cache can be off here).
     *
     * rt_irq_stack_top[0] is still __stack_end__, so CPU0 is unaffected.
     */
    la    t2, rt_irq_stack_top
    csrr  t0, mhartid
    slli  t0, t0, 2
    add   t0, t0, t2
    lw    sp, 0(t0)
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
    /* This core's interrupt nest counter.
     *
     * The single-core build reads the global directly: one `la` and one load,
     * nothing that can land in flash, and this exact shape has been running
     * since the first bring-up.
     *
     * The SMP build cannot use a global there - rt_interrupt_nest is a macro
     * over rt_cpu_self()->irq_nest (irq.c:68) and has no symbol - so it indexes
     * an IRAM table of per-CPU pointers with mhartid. It cannot use a call
     * either: IDF disables the flash cache while tuning MSPI/PSRAM timing, and a
     * flash-resident callee faults in that window (observed: MEPC 0x40013b1e
     * resolved to rt_enter_critical in IROM).
     *
     * The two builds therefore read this value differently on purpose. */
    la    t2, rt_irq_nest_slot
    csrr  t0, mhartid
    slli  t0, t0, 2
    add   t0, t0, t2
    lw    t0, 0(t0)         /* t0 = slot[n], the address of this core's counter */
    lw    t0, 0(t0)         /* t0 = the counter itself - the branch needs a value,
                             * not a pointer: a pointer is never zero, so a missing
                             * load here makes every interrupt look nested */
    bnez  t0, spurious_interrupt

#ifdef RT_USING_SMP
    /*
     * SMP: the kernel decides and performs the switch.
     *
     * There is no "switch requested" flag to consult - two cores sharing one
     * flag would race, and the MP scheduler already keeps the decision in the
     * per-CPU structure. rt_scheduler_do_irq_switch() takes the trap frame as
     * its argument and rt_hw_context_switch_exit restores from the stack it
     * leaves behind, which is exactly the shape the reference port uses
     * (interrupt_gcc.S:312-340).
     */
    mv    sp, sp
    mv    a0, sp
    call  rt_scheduler_do_irq_switch
    tail  rt_hw_context_switch_exit
#else
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

#endif /* RT_USING_SMP */

spurious_interrupt:
    /* Restore the return address from the selected context for nesting or thread switching. */
    LOAD  a0,  0 * REGBYTES(sp)
    csrw  mepc, a0

    LOAD  x1,   1 * REGBYTES(sp)

    LOAD  t0,   2 * REGBYTES(sp)
    /* Supply MPP=M and, on FPU targets, FS=3.
     *
     * Both are missing from a frame produced by a cooperative switch, because
     * port/common/context_gcc.S stores only MPIE in slot 2 - it masks mstatus
     * with 8 first (context_gcc.S:152-158). So whatever slot 2 holds, the two
     * fields a trap exit needs have to be added here.
     *
     * MPP is why an mret on such a frame would otherwise return to user mode
     * and fault. Every target this file is built for (C3, C6, S31) is
     * machine-mode only, and a capability predicate would be wrong: S31 sets
     * SOC_INT_HW_NESTED_SUPPORTED yet needs the fix exactly like the PLIC
     * targets do.
     *
     * FS is the one that actually bit us. This file restores f0..f31 with flw
     * after this point, and executing an FP instruction with mstatus.FS=0 is an
     * illegal instruction. On ESP32-S31 that is reported as MCAUSE=1
     * (instruction access fault) with MTVAL equal to the flw's own address,
     * because SOC_CPU_HAS_FPU_EXT_ILL_BUG means the EXT_ILL CSR does not record
     * FLW/FSW (soc_caps.h:184) and vectors.S has no rev<3 workaround for this
     * target. The observed crash was exactly that: MEPC=MTVAL=the first flw of
     * this epilogue, MSTATUS=0x80006080, i.e. FS read back as 3 at fault time
     * while the mstatus written one instruction earlier had it at 0.
     *
     * context_gcc.S:245 already writes 0x7800 for FPU builds; this is the same
     * value, so both exits now agree on what a resumed frame looks like.
     */
#ifdef RT_FPU_FRAME
    li    t1, 0x7800                        /* MPP=M | FS=3 */
#else
    li    t1, 0x1800                        /* MPP=M */
#endif
    or    t0, t0, t1
    csrw  mstatus, t0                       /* Restore the saved interrupt state, supplying MPP (and FS on FPU targets). */

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

#ifdef RT_FPU_FRAME
    /* Paired with the prologue. The mstatus restored above carries FS=3 for
     * every frame that has FP slots, so these loads do not trap. */
    FPLOAD f0,  ( 0 * FPREG_BYTES)(sp)
    FPLOAD f1,  ( 1 * FPREG_BYTES)(sp)
    FPLOAD f2,  ( 2 * FPREG_BYTES)(sp)
    FPLOAD f3,  ( 3 * FPREG_BYTES)(sp)
    FPLOAD f4,  ( 4 * FPREG_BYTES)(sp)
    FPLOAD f5,  ( 5 * FPREG_BYTES)(sp)
    FPLOAD f6,  ( 6 * FPREG_BYTES)(sp)
    FPLOAD f7,  ( 7 * FPREG_BYTES)(sp)
    FPLOAD f8,  ( 8 * FPREG_BYTES)(sp)
    FPLOAD f9,  ( 9 * FPREG_BYTES)(sp)
    FPLOAD f10, (10 * FPREG_BYTES)(sp)
    FPLOAD f11, (11 * FPREG_BYTES)(sp)
    FPLOAD f12, (12 * FPREG_BYTES)(sp)
    FPLOAD f13, (13 * FPREG_BYTES)(sp)
    FPLOAD f14, (14 * FPREG_BYTES)(sp)
    FPLOAD f15, (15 * FPREG_BYTES)(sp)
    FPLOAD f16, (16 * FPREG_BYTES)(sp)
    FPLOAD f17, (17 * FPREG_BYTES)(sp)
    FPLOAD f18, (18 * FPREG_BYTES)(sp)
    FPLOAD f19, (19 * FPREG_BYTES)(sp)
    FPLOAD f20, (20 * FPREG_BYTES)(sp)
    FPLOAD f21, (21 * FPREG_BYTES)(sp)
    FPLOAD f22, (22 * FPREG_BYTES)(sp)
    FPLOAD f23, (23 * FPREG_BYTES)(sp)
    FPLOAD f24, (24 * FPREG_BYTES)(sp)
    FPLOAD f25, (25 * FPREG_BYTES)(sp)
    FPLOAD f26, (26 * FPREG_BYTES)(sp)
    FPLOAD f27, (27 * FPREG_BYTES)(sp)
    FPLOAD f28, (28 * FPREG_BYTES)(sp)
    FPLOAD f29, (29 * FPREG_BYTES)(sp)
    FPLOAD f30, (30 * FPREG_BYTES)(sp)
    FPLOAD f31, (31 * FPREG_BYTES)(sp)
    addi  sp, sp, 32 * FPREG_BYTES
#endif

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
