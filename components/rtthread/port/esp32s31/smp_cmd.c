/*
 * s31smp - report the per-CPU state the SMP work depends on.
 *
 * P3 has been a sequence of "the platform side the MP model assumes is not there
 * yet": per-CPU structures, an interrupt stack per core, a tick per core, threads
 * bound to a core. Each was established by reading source or by a one-off probe,
 * and then the probe was removed. That makes every later step re-derive facts that
 * were already measured.
 *
 * This command makes the measurable ones permanent. It reads only public state -
 * struct rt_cpu, the stack-top table - and prints it per core, so the next round
 * can check its assumptions in one command instead of re-reading the kernel.
 *
 * What it deliberately does NOT claim: whether CPU1 is scheduling. That is not
 * visible from these fields on CPU0 - a core with no thread running still has a
 * current_thread of NULL and a tick of 0, which is exactly the state a parked core
 * is in. The cpu1/cpu1lk markers from smp_slave*.c are the evidence for that, and
 * s31stat reports those.
 */

#include <rthw.h>
#include <rtthread.h>

#include "cpuport.h"

#ifdef RT_USING_SMP

/* Defined in smp_istack.c: top of each core's interrupt stack. */
extern void *rt_irq_stack_top[RT_CPUS_NR];

#ifdef RT_SMP_CPU1_SCHEDULES

/*
 * Proof that CPU1 runs threads, not just that it entered the scheduler.
 *
 * `rt_system_scheduler_start()` on CPU1 fills in that core's current_thread, so
 * s31smp's cur_thread column showing the idle thread only proves the call was reached.
 * A thread bound to CPU1 changes state every loop and increments a counter; if the
 * counter moves, CPU1 is dispatching.
 */
static volatile unsigned s31_cpu1_loops;
static volatile unsigned s31_cpu1_seen_on;

static void s31_cpu1_probe_thread(void *parameter)
{
    RT_UNUSED(parameter);

    /* Which core am I on? This is the only place the answer is trustworthy: the shell
     * runs on CPU0 and can only report what the structures say. */
    s31_cpu1_seen_on = (unsigned)rt_hw_cpu_id();

    while (1)
    {
        s31_cpu1_loops++;
        rt_thread_mdelay(10);
    }
}

static int s31_cpu1_probe(void)
{
    rt_thread_t t;
    unsigned before;

    s31_cpu1_loops = 0;
    s31_cpu1_seen_on = 0xFFFFFFFFu;

    t = rt_thread_create("cpu1probe", s31_cpu1_probe_thread, RT_NULL,
                         2048, 20, 10);
    if (t == RT_NULL)
    {
        return -1;
    }

    rt_thread_control(t, RT_THREAD_CTRL_BIND_CPU, (void *)1);
    rt_thread_startup(t);

    rt_thread_mdelay(300);
    before = s31_cpu1_loops;

    rt_kprintf("  cpu1probe ran on cpu %u, %u loops in 300 ms\n",
               s31_cpu1_seen_on, before);

    return (before > 0) ? 0 : -1;
}

#endif /* RT_SMP_CPU1_SCHEDULES */

static int cmd_smp(int argc, char **argv)
{
    int cpu;
    void *seen_stack[RT_CPUS_NR];
    int distinct = 1;

    (void)argc;
    (void)argv;

    rt_kprintf("cpus   : %d, running on cpu %d\n", RT_CPUS_NR, (int)rt_hw_cpu_id());
    rt_kprintf("\n");
    rt_kprintf("cpu cur_thread idle_thread irq_nest tick  istack\n");
    rt_kprintf("--- ---------- ----------- -------- ----- --------\n");

    for (cpu = 0; cpu < RT_CPUS_NR; cpu++)
    {
        struct rt_cpu *pcpu = rt_cpu_index(cpu);

        seen_stack[cpu] = rt_irq_stack_top[cpu];

        rt_kprintf("%3d %10p %11p %8d %5d %p\n",
                   cpu,
                   (void *)pcpu->current_thread,
                   (void *)pcpu->idle_thread,
                   (int)pcpu->irq_nest,
                   (int)pcpu->tick,
                   rt_irq_stack_top[cpu]);
    }

    /* Two cores sharing one interrupt stack is the failure this table exists to
     * prevent, so check it rather than leaving it to whoever reads the addresses. */
    for (cpu = 1; cpu < RT_CPUS_NR; cpu++)
    {
        if (seen_stack[cpu] == seen_stack[0])
        {
            distinct = 0;
        }
    }

    rt_kprintf("\n");
    rt_kprintf("istack : %s\n", distinct ? "distinct per core" : "SHARED - fix smp_istack.c");

#ifdef RT_SMP_CPU1_SCHEDULES
    if (s31_cpu1_probe() == 0)
    {
        rt_kprintf("cpu1   : SCHEDULING (a bound thread is running there)\n");
    }
    else
    {
        rt_kprintf("cpu1   : entered the scheduler but ran nothing\n");
    }
#endif

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_smp, s31smp, show per-CPU SMP state);

/*
 * Anchor so the linker keeps this file.
 *
 * MSH_CMD_EXPORT only puts a record in a section; nothing references the object,
 * so with -ffunction-sections/--gc-sections the whole thing is dropped and the
 * command silently does not exist. board.c calls this. The alternative - adding
 * -u for a symbol nobody uses - would be worse: the call documents the intent.
 */
void s31_smp_cmd_link_anchor(void)
{
}

#endif /* RT_USING_SMP */
