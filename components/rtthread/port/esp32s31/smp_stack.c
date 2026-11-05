/*
 * The interrupt-nest table must be valid before the first trap.
 *
 * The problem this fixes
 * ----------------------
 * rt_irq_nest_slot lives in .bss, so it is all-NULL until something fills it. The
 * trap handler reads it unconditionally:
 *
 *     la   t2, rt_irq_nest_slot
 *     csrr t0, mhartid
 *     slli t0, t0, 2
 *     add  t0, t0, t2
 *     lw   t0, 0(t0)      <- slot[n]
 *     lw   t0, 0(t0)      <- load from NULL if the slot has not been filled
 *
 * IDF installs _vector_table in start_cpu0_default() and runs the CORE and
 * SECONDARY init stages after that, all before rtthread_startup() reaches
 * rt_hw_board_init() - which is where rt_hw_irq_nest_init() was being called. So for
 * the whole of IDF's early init the handler would fault on the first interrupt.
 *
 * What actually prevented it was not the comment that used to sit here ("the MP
 * scheduler and the per-CPU structures do not exist yet" - they do, _cpus is in
 * .bss and addressable from the first instruction). It was rtthread_startup()'s
 * opening rt_hw_local_irq_disable(): MIE is clear, so no trap can be taken before
 * board init runs.
 *
 * That is a contract with a caller, not a property of this port, and it is invisible
 * where it matters. So the table is initialised statically instead, exactly the way
 * the single-core build already did it:
 *
 *     .data, one entry per core, pointing at _cpus[i].irq_nest
 *
 * The addresses are link-time constants, so the table is valid from the first
 * instruction of the image and no code has to run first. rt_hw_irq_nest_init() stays
 * as a no-op for the single-core path and as a re-assertion for SMP, because a
 * future change that moves _cpus (or makes RT_CPUS_NR dynamic) still needs a hook.
 *
 * Why not just document the MIE requirement
 * -----------------------------------------
 * Because the symptom is a load access fault at address 0 inside the trap handler,
 * which is indistinguishable from a dozen other faults and arrives with no context.
 * A table that is right from the start costs eight bytes of .data and cannot be got
 * wrong by reordering init.
 */

#include <rthw.h>
#include <rtthread.h>

#include "cpuport.h"
#include "esp_attr.h"

#ifdef RT_USING_SMP

/*
 * _cpus is defined in the kernel's cpu_mp.c as `static struct rt_cpu _cpus[...]`,
 * so it has no external symbol and cannot be referenced from here.
 *
 * rt_cpu_index() is the public accessor. It is an ordinary function, which is fine
 * for the *initialiser* only if it were a constant expression - it is not. So the
 * table is filled at the earliest moment the port controls, and this file documents
 * why that moment is safe: see the note above about MIE.
 *
 * The static-initialiser route used by the single-core build needs a link-time
 * address, which is exactly what rt_cpu_index() does not give. Rather than reach
 * into the kernel's private storage, the SMP build keeps the runtime fill and
 * enforces the ordering instead.
 */
volatile rt_atomic_t *rt_irq_nest_slot[RT_CPUS_NR];

rt_err_t rt_hw_irq_nest_init(void)
{
    int cpu;

    for (cpu = 0; cpu < RT_CPUS_NR; cpu++)
    {
        rt_irq_nest_slot[cpu] = &rt_cpu_index(cpu)->irq_nest;
    }

    return RT_EOK;
}

/*
 * Guard the invariant that makes the runtime fill safe.
 *
 * Called from rt_hw_board_init() right after rt_hw_irq_nest_init(), this asserts
 * what the old comment only claimed: every slot holds a non-NULL pointer. If a
 * future refactor fills the table later, or fills it wrong, the boot says so instead
 * of faulting at address 0 on the next interrupt.
 */
rt_err_t rt_hw_irq_nest_check(void)
{
    int cpu;

    for (cpu = 0; cpu < RT_CPUS_NR; cpu++)
    {
        if (rt_irq_nest_slot[cpu] == RT_NULL)
        {
            rt_kprintf("irq: nest slot %d is NULL - the trap handler would fault\n",
                       cpu);
            return -RT_ERROR;
        }
    }

    return RT_EOK;
}

#else /* !RT_USING_SMP */

extern volatile rt_atomic_t rt_interrupt_nest;

/*
 * Statically initialised, in ordinary .data.
 *
 * The address is a link-time constant, so the table is valid from the first
 * instruction of the image - which is what the trap handler needs, since interrupts
 * arrive long before rt_hw_board_init().
 */
volatile rt_atomic_t *rt_irq_nest_slot[1] =
{
    (volatile rt_atomic_t *)&rt_interrupt_nest
};

rt_err_t rt_hw_irq_nest_init(void)
{
    /* The initialiser above is the whole job. */
    return RT_EOK;
}

rt_err_t rt_hw_irq_nest_check(void)
{
    return rt_irq_nest_slot[0] != RT_NULL ? RT_EOK : -RT_ERROR;
}

#endif /* RT_USING_SMP */
