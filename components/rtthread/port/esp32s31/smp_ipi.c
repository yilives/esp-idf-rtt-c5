/*
 * IPI delivery for the S31 (inter-processor interrupt).
 *
 * The SMP scheduler uses this to tell another core that its run queue changed
 * (`RT_SCHEDULE_IPI`). On this part the mechanism is the same line IDF reserves
 * for crosscore traffic: ETS_IPC_ISR_INUM (28, soc.h), delivered through the
 * CLIC.
 *
 * Deliberately minimal: only the send side, and even that is a no-op until a receive
 * side exists. The receive side needs an ISR in the RT-Thread vector table, and wiring
 * that half-way would leave an interrupt that fires with nobody to service it.
 *
 * Because of this gap, binding a RUNNING thread to core 1 stops that thread: the local
 * reschedule in rt_sched_thread_bind_cpu() takes it off core 0's ready queue and core 1
 * never schedules it. Measured controls are in rt_hw_ipi_send() below. `bind` is an
 * upstream msh builtin, so the honest description is "do not bind a running thread to
 * core 1 on this port until the IPI receive side and CPU1 scheduling exist".
 */

#include <rthw.h>
#include <rtthread.h>

#include "riscv/interrupt.h"

#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/interrupts.h"
#include "soc/clic_reg.h"
#include "riscv/csr_clic.h"

#ifdef RT_USING_SMP

/*
 * Set when an ISR exists for the crosscore line. Until then rt_hw_ipi_send() is a
 * no-op, because asserting a pending bit that nothing services is worse than not
 * asserting it: the sender waits for a reschedule that cannot come.
 *
 * Nothing sets this yet. It is a named place for the state rather than an #ifdef so
 * that adding the receive side is a one-line change here plus the ISR itself.
 */
static int s31_ipi_receive_side_ready = 0;

/*
 * The kernel's IPI handler. `rt_scheduler_ipi_handler()` is provided by
 * scheduler_mp.c and does exactly what is needed: rt_schedule(), which is safe to call
 * from an ISR - it sets irq_switch_flag and lets the trap exit do the switch.
 */
extern void rt_scheduler_ipi_handler(int vector, void *param);

static void s31_ipi_isr(int vector, void *param)
{
    rt_scheduler_ipi_handler(vector, param);
}

/*
 * Install the receive side. Called once, from CPU0, before CPU1 is released: the line
 * is shared, so one install covers both cores.
 *
 * Nothing else in this image owns ETS_IPC_ISR_INUM; `s31intr` lists it unhandled before
 * this runs and handled afterwards, which is the check that it took.
 */
rt_err_t rt_hw_ipi_handler_install(int vector, rt_isr_handler_t handler);
rt_err_t rt_hw_ipi_handler_install(int vector, rt_isr_handler_t handler)
{
    (void)vector;
    (void)handler;

    if (rt_hw_interrupt_install(ETS_IPC_ISR_INUM, s31_ipi_isr, RT_NULL,
                                "crosscore") == RT_NULL)
    {
        /* install() returns NULL both for "slot was empty" and for "vector rejected",
         * so ask the slot instead of trusting the return value. */
        if (intr_handler_get(ETS_IPC_ISR_INUM) == RT_NULL)
        {
            rt_kprintf("ipi: install failed on INUM %d\n", ETS_IPC_ISR_INUM);
            return -RT_ERROR;
        }
    }

    rt_hw_interrupt_umask(ETS_IPC_ISR_INUM);

    s31_ipi_receive_side_ready = 1;

    return RT_EOK;
}

/*
 * Raise the crosscore interrupt.
 *
 * "Pending" on the CLIC is a bit in the same per-line control word that holds
 * the enable and the trigger type, so it has to be set with a read-modify-write:
 * a plain store of CLIC_INT_IP would clear the priority, the enable and the
 * trigger along with it. clic_reg.h: "CLIC_INT_IP : R/W ;bitpos:[0]" and
 * "CLIC_INT_IE : R/W ;bitpos:[8]" both live in CLIC_INT_CTRL_REG(i).
 */
void rt_hw_ipi_send(int ipi_vector, unsigned int cpu_mask)
{
    volatile rt_uint32_t *reg;
    rt_base_t level;

    /* The S31 has one crosscore line, not a vector table of mailboxes, so the
     * vector number is advisory and only the mask decides whether to fire. */
    (void)ipi_vector;

    /*
     * Refuse to raise a line nobody services.
     *
     * There is no receive side (see the file header), so the pending bit would be
     * asserted and never cleared. That is wrong on its own terms - an asserted line
     * with no ISR is an interrupt storm waiting for the moment the line is unmasked.
     *
     * This is NOT what makes `bind <thread> 1` wedge the shell. That was tested:
     * with this guard in place the shell still wedges, every time, so the IPI was
     * not the mechanism. The mechanism is one level up, in
     * rt_sched_thread_bind_cpu():
     *
     *     rt_hw_ipi_send(RT_SCHEDULE_IPI, 1U << cpu);
     *     rt_sched_unlock_n_resched(slvl);      <- reschedules CPU0
     *
     * The second call reschedules the CORE THAT IS RUNNING the thread. Once the
     * thread is bound to CPU1 it is gone from CPU0's ready queue, CPU0 picks
     * something else, and CPU1 - parked in wfi, never scheduling - never picks it up.
     * The thread is not stuck in the kernel; it simply never gets the CPU again.
     *
     * Measured controls, all on one image:
     *
     *     bind <shell>   0  -> works (cpu == current_cpu branch, no IPI, no resched)
     *     bind <tsystem> 1  -> works (thread not RUNNING, so neither path is taken)
     *     bind <shell>   1  -> wedges, with and without this guard
     */
    if (!s31_ipi_receive_side_ready)
    {
        return;
    }

    if ((cpu_mask & ((1u << RT_CPUS_NR) - 1u)) == 0u)
    {
        return;
    }

    reg = (volatile rt_uint32_t *)CLIC_INT_CTRL_REG(
        ETS_IPC_ISR_INUM + RV_EXTERNAL_INT_OFFSET);

    level = rt_hw_interrupt_disable();
    *reg |= CLIC_INT_IP;
    rt_hw_interrupt_enable(level);
}

#endif /* RT_USING_SMP */
