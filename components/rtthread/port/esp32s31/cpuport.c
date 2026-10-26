/*
 * ESP32-S31 CPU port for the ESP-IDF RT-Thread component.
 *
 * Derived from port/esp32c6/cpuport.c. The target is the same RV32 machine-mode
 * ABI, so the only differences are the SoC name and the interrupt-nesting note
 * below. rt_hw_stack_init_core() keeps its name because the shared
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

/* S31 adaptation: only mask/restore MIE here.
 *
 * ESP32-S31 has a CLIC with hardware nesting (SOC_INT_HW_NESTED_SUPPORTED), so
 * unlike C3/C6 the trap path in vectors_rtthread.S does NOT save, raise and
 * restore the interrupt threshold in software - mintstatus is pushed by the
 * CLIC on entry and popped on mret. rt_hw_interrupt_disable() therefore only
 * has to clear MIE, which is exactly what context_gcc.S does.
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

    /* Keep the first source and the latest destination across nested IRQs.
     * The outermost S31 trap return consumes this request; nested returns
     * restore their own mcause and go back to the interrupted handler.
     */
    rt_hw_interrupt_enable(level);
}
#endif /* end of RT_USING_SMP */
