/*
 * ESP32-C5 CPU port for the ESP-IDF RT-Thread component.
 *
 * Derived from port/esp32c6/cpuport.c. Both chips are RV32IMAC machine mode
 * with a CLIC and one application core, so the only differences are the SoC
 * name, the interrupt-nesting note below and the two trap-path tables at the
 * bottom of this file.
 *
 * rt_hw_stack_init_core() keeps its name because the shared
 * rtthread_stack_init.c adapter calls it.
 */
/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/10/28     Bernard      The unify RISC-V porting code.
 * 2020/11/20     BalanceTWK   Add FPU support
 * 2023/01/04     WangShun     Adapt to CH32
 */

#include <rthw.h>
#include <rtthread.h>

#include "cpuport.h"
#include "rt_hw_stack_frame.h"

#ifndef RT_USING_SMP
volatile rt_ubase_t  rt_interrupt_from_thread = 0;
volatile rt_ubase_t  rt_interrupt_to_thread   = 0;
volatile rt_uint32_t rt_thread_switch_interrupt_flag = 0;
#endif

/*
 * The two tables vectors_rtthread.S indexes on every trap.
 *
 * The trap entry used to read the global rt_interrupt_nest directly and to load
 * the literal address __stack_end__. It now goes through tables indexed by
 * mhartid, because the same assembly serves the two-core S31. On a one-core SoC
 * that indirection has to be satisfied too, or the link fails with undefined
 * references from an assembly file - which is a confusing place to end up.
 *
 * Both arrays are in .data, not .rodata: the trap handler reads them while the
 * flash cache may be off (an SPI flash write, or MSPI timing tuning), and a
 * flash-resident table faults in exactly that window.
 */
extern volatile rt_atomic_t rt_interrupt_nest;
extern char __stack_end__;

volatile rt_atomic_t *rt_irq_nest_slot[RT_CPUS_NR] =
{
    &rt_interrupt_nest,
};

void *rt_irq_stack_top[RT_CPUS_NR] =
{
    &__stack_end__,
};

/**
 * This function will initialize thread stack
 *
 * @param tentry the entry of thread
 * @param parameter the parameter of entry
 * @param stack_addr the beginning stack address
 * @param texit the function will be called when thread exit
 *
 * @return stack address
 */
rt_uint8_t *rt_hw_stack_init_core(void       *tentry,
                             void       *parameter,
                             rt_uint8_t *stack_addr,
                             void       *texit)
{
    struct rt_hw_stack_frame *frame;
    rt_uint8_t         *stk;
    int                i;

    stk  = stack_addr + sizeof(rt_ubase_t);
    stk  = (rt_uint8_t *)RT_ALIGN_DOWN((rt_ubase_t)stk, REGBYTES);
    stk -= sizeof(struct rt_hw_stack_frame);

    frame = (struct rt_hw_stack_frame *)stk;

    for (i = 0; i < sizeof(struct rt_hw_stack_frame) / sizeof(rt_ubase_t); i++)
    {
        ((rt_ubase_t *)frame)[i] = 0xdeadbeef;
    }

    frame->ra      = (rt_ubase_t)texit;
    frame->a0      = (rt_ubase_t)parameter;
    frame->epc     = (rt_ubase_t)tentry;

    /* force to machine mode(MPP=11) and set MPIE to 1 */
#ifdef ARCH_RISCV_FPU
    frame->mstatus = 0x7880;
#else
    frame->mstatus = 0x1880;
#endif

    return stk;
}

/* C5 adaptation: only mask/restore MIE.
 *
 * ESP32-C5 has a CLIC with hardware nesting (soc_caps.h defines
 * SOC_INT_CLIC_SUPPORTED and SOC_INT_HW_NESTED_SUPPORTED), so unlike the C3 the
 * trap path in vectors_rtthread.S does not save, raise and restore the
 * interrupt threshold in software - mintstatus is pushed by the CLIC on entry
 * and popped on mret. rt_hw_interrupt_disable() therefore only has to clear
 * MIE, which is exactly what context_gcc.S does.
 *
 * Note that the C6 does NOT take this path despite having a CLIC: it has no
 * hardware nesting, so its threshold handling stays in software. The predicate
 * is SOC_INT_HW_NESTED_SUPPORTED, read from soc_caps.h, never a target name.
 */
rt_bool_t rt_hw_interrupt_is_disabled(void)
{
    rt_ubase_t level;
    __asm__ volatile ("csrr %0, mstatus" : "=r" (level) :: "memory");
    return (level & 8) == 0;
}

/*
 * #ifdef RT_USING_SMP
 * void rt_hw_context_switch_interrupt(void *context, rt_ubase_t from, rt_ubase_t to, struct rt_thread *to_thread);
 * #else
 * void rt_hw_context_switch_interrupt(rt_ubase_t from, rt_ubase_t to);
 * #endif
 */
#ifndef RT_USING_SMP
void rt_hw_context_switch_interrupt(rt_ubase_t from, rt_ubase_t to, rt_thread_t from_thread, rt_thread_t to_thread)
{
    rt_base_t level = rt_hw_interrupt_disable();
    RT_UNUSED(from_thread);
    RT_UNUSED(to_thread);
    if (rt_thread_switch_interrupt_flag == 0)
        rt_interrupt_from_thread = from;

    rt_interrupt_to_thread = to;
    rt_thread_switch_interrupt_flag = 1;

    /* Keep the first source and the latest destination across nested IRQs. The
     * outermost trap return consumes this request; nested returns restore their
     * own mcause and go back to the interrupted handler. There is no
     * software-interrupt poke here because the switch is picked up on the way
     * out of the interrupt that raised it. */
    rt_hw_interrupt_enable(level);
}
#endif /* end of RT_USING_SMP */
