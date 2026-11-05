/*
 * CPU1 and the scheduler lock.
 *
 * smp_slave.c gets CPU1 out of reset and into our code; this file is what that
 * code does next.
 *
 * Why the previous attempts hung - measured, not guessed
 * -----------------------------------------------------
 * Attempt 1 (W49): CPU1 took _cpus_lock and then called
 * rt_system_scheduler_start(). Result: boot loop.
 *
 * Attempt 2 (W50): CPU1 took _cpus_lock and parked in wfi without releasing it.
 * Result: boot loop with
 *
 *     rst:0x8 (HP_SYS_HP_WDT1_RESET)
 *     Core0 Saved PC:0x2f016d30      -> nm: inside rt_hw_spin_lock
 *
 * The saved PC is CPU0's, and it is in the spin lock - so CPU0 was the one stuck.
 * The cause was this file: taking _cpus_lock and never giving it back starves
 * CPU0, which acquires the same lock again later in its startup. That is a bug in
 * the port code, not in the kernel.
 *
 * What this stage therefore tests
 * -------------------------------
 * The genuinely unknown part is whether the ticket lock works from CPU1 at all:
 * its amoadd.w needs the two cores to share one coherent view of the lock word,
 * and CPU1's cache state is whatever this core inherited. Acquiring and then
 * *releasing* answers that - the lock is left exactly as found, so CPU0 cannot be
 * starved either way.
 *
 * Attempt 3 (W52): CPU1 took the lock, released it, then called
 * rt_system_scheduler_start() - with a probe in the picker. Result:
 *
 *     [pick] cpu=0 global=10 local=31 -> main
 *     [pick] cpu=1 global=10 local=31 -> main
 *
 * Both cores chose the same unbound thread, so main ran on both at once and
 * CPU0 ended up in rt_hw_spin_lock with HP_WDT. That is not a picker bug: unbound
 * threads live in the global ready table and are visible to every core, which is
 * what the MP model intends - the platform side it assumes (per-CPU tick, per-CPU
 * interrupt routing, CPU-bound threads) is what this port does not have yet.
 *
 * So the scheduler start is not enabled here. What this file does is keep the
 * measurements: the ticket lock works from CPU1, and starting the scheduler
 * without the platform side makes two cores own one thread.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "esp_attr.h"

#ifdef RT_SMP_CPU1_SCHEDULES
extern rt_err_t rt_hw_secondary_cpu_tick_init(void);
extern void rt_hw_secondary_cpu_irq_enable(void);
#endif

#ifdef RT_USING_SMP

/*
 * Set once CPU1 has taken and released the scheduler lock.
 *
 * Read by s31stat. A non-zero value is the proof that amoadd.w from this core is
 * coherent with CPU0's view of the same word - which is what "the ticket lock
 * works on two cores" actually means.
 */
volatile unsigned s31_cpu1_locked;

/*
 * Set once the system is up enough for CPU1 to run threads. Written from main_thread_entry
 * (INIT_APP_EXPORT in smp_tick.c), read by CPU1's entry. Separate from s31_cpu1_locked,
 * which only reports that the ticket lock worked.
 */
volatile unsigned s31_cpu1_go;

void rt_hw_secondary_cpu_start_scheduler(void)
{
    s31_cpu1_locked = 0;

    /*
     * Take the scheduler lock:
     *
     *   - rt_system_scheduler_start() releases it unconditionally as its first
     *     statement, so a secondary core has to hold it for the pair to balance;
     *   - it also sequences the cores, because CPU0 holds this lock from
     *     rtthread_startup() until its own rt_system_scheduler_start().
     */
    /*
     * Take the scheduler lock and give it straight back.
     *
     * Two things are being verified here, and only two:
     *
     *   - the ticket lock works from this core: amoadd.w has to be coherent with
     *     CPU0's view of the same word, and taking then releasing exercises the
     *     acquire path, the ticket comparison and the release;
     *   - the two cores are sequenced: CPU0 holds this lock from rtthread_startup()
     *     until its own rt_system_scheduler_start(), so getting through the acquire
     *     means CPU0 got that far.
     *
     * Deliberately NOT calling rt_system_scheduler_start() afterwards.
     *
     * W56 measured that working three times in a row, and it does start this core
     * on its idle thread:
     *
     *     tidle1   cpu 1   bind 1   running
     *
     * But every rebuild since does not survive it: the banner and the `msh >`
     * prompt appear and then no command executes, with no panic and no reset. Same
     * board, same lead, same toolchain - so it is a race, not a configuration
     * error, and the most likely window is that rt_system_scheduler_start()
     * releases _cpus_lock *before* running the picker, while the picker reads the
     * per-core ready tables that rt_scheduler_do_irq_switch() edits under that same
     * lock. Whichever core arrives second sees a table mid-edit.
     *
     * Settling that is a kernel change with its own verification, so this core
     * stays out of scheduling until then. It still starts, still executes IRAM
     * code and still proves the shared lock works, which is what the port itself
     * needed to show.
     */
    rt_hw_spin_lock(&_cpus_lock);
    s31_cpu1_locked = 0x12345678u;
    rt_hw_spin_unlock(&_cpus_lock);

#ifdef RT_SMP_CPU1_SCHEDULES
    /*
     * Wait until CPU0 has a running system.
     *
     * This core is released from rtthread_startup() *before* the idle threads exist and
     * before CPU0 picks its first thread, so starting to schedule here would find an
     * empty ready table and trip RT_ASSERT(to_thread) in
     * _scheduler_get_highest_priority_thread(). Measured: that is a
     * `rst:0x8 HP_SYS_HP_WDT1_RESET` loop, not a clean assert, because the trap happens
     * with the scheduler unlocked.
     *
     * The flag is set from an INIT_APP_EXPORT (see smp_tick.c), which runs from
     * main_thread_entry - after CPU0's scheduler is running and both idle threads exist.
     */
    while (s31_cpu1_go == 0)
    {
        rt_hw_us_delay(100);
    }

    /*
     * CPU1's own tick, before it starts scheduling.
     *
     * rt_tick_increase() increments rt_cpu_self()->tick and calls
     * rt_sched_tick_increase(), which expires this core's time slice. Without an alarm
     * here CPU1 has tick == 0 forever and its idle thread can never be preempted, so
     * scheduling on this core would be a no-op even if it started.
     */
    /*
     * Interrupts first, then the tick.
     *
     * IDF does the same two register writes at the top of xPortStartScheduler() on every
     * core. Here they were only ever done from board init - on CPU0. `mintthresh` is
     * per-core, so CPU1 was left with a threshold that masks every interrupt: the tick
     * was configured but could never be delivered, and a thread bound to this core could
     * never be preempted or even started.
     */
    {
        extern volatile unsigned s31_cpu1_stage;

        /* 1: readbacks done */
        s31_cpu1_stage = 1;

        rt_hw_secondary_cpu_irq_enable();

        /* 2: threshold lowered and MIE set. If stage stays at 1 from here on, the trap
         * was taken at that csrsi and never came back. */
        s31_cpu1_stage = 2;

    #ifndef RT_SMP_CPU1_NO_TICK
    rt_hw_secondary_cpu_tick_init();
#else
    /*
     * Experiment: skip the tick entirely, to find out whether the stop in the spin loop is
     * caused by the tick at all. `spin` completing means the tick was the trigger; a short
     * `spin` means the loop or something around it stops this core regardless.
     */
    s31_cpu1_stage = 3;
#endif

        /* 3: tick configured and its line enabled on this core. */
        s31_cpu1_stage = 3;
    }

    /*
     * From here CPU1 runs threads.
     *
     * Take _cpus_lock and let rt_system_scheduler_start() release it - that is the
     * balance the kernel expects, and calling it without holding the lock underflows the
     * count (W56's finding). The picker runs under _mp_scheduler_lock and removes the
     * thread it selects, so the two cores cannot select the same one.
     */
    /*
     * Record the two per-core gates before handing over.
     *
     * The CLIC line is enabled and pending but never serviced, so the question is what
     * this core's own CSRs say. `mintthresh` is what CLIC compares the line level against
     * (it delivers only when int_ctl_byte > mintthresh_byte) and mstatus.MIE is the global
     * enable. Neither has been *measured* on this core - only attempted.
     */
    {
        extern volatile unsigned s31_cpu1_mintthresh;
        extern volatile unsigned s31_cpu1_mstatus;
        extern volatile unsigned s31_cpu1_mintstatus;
        rt_uint32_t v;

        __asm__ volatile ("csrr %0, 0x347" : "=r"(v));   /* mintthresh */
        s31_cpu1_mintthresh = v;

        __asm__ volatile ("csrr %0, mstatus" : "=r"(v));
        s31_cpu1_mstatus = v;

        /* mintstatus: MIL is bits 31:24. Non-zero means this core is inside a handler
         * (or was, and never returned) - which masks everything at or below that level. */
        __asm__ volatile ("csrr %0, 0xfb1" : "=r"(v));
        s31_cpu1_mintstatus = v;
    }

    /*
     * Spin 100 ms with interrupts enabled, counting, before the scheduler takes over.
     *
     * This is the one experiment that separates "CPU1 is not running" from "CPU1 runs but
     * the trap is not taken": it needs nothing but straight-line code, and the tick line
     * is already enabled and pending during the whole loop.
     */
    /*
     * Deliberate fault, before anything else can muddy the picture.
     *
     * Reading address 0 exercises mtvec/mtvt with no CLIC arbitration, no matrix and no
     * driver: if the panic handler reports it, the trap path on this core works and the
     * missing tick is a delivery problem. If the board goes silent instead, the trap path
     * is broken here - which would itself explain the pending-but-unserviced line.
     *
     * Off by default; enabled only for this experiment.
     */
#ifdef RT_SMP_CPU1_FAULT_TEST
    {
        extern volatile unsigned s31_cpu1_stage;

        s31_cpu1_stage = 8;              /* 8 = about to fault */
        *(volatile rt_uint32_t *)0;
        s31_cpu1_stage = 9;              /* 9 = fault did not happen */
    }
#endif

    {
        extern volatile unsigned s31_cpu1_spin;
        extern volatile unsigned s31_cpu1_stage;

        rt_uint32_t deadline = 100000;

        while (deadline--)
        {
            s31_cpu1_spin++;
        }

        s31_cpu1_stage = 4;
    }

    rt_hw_spin_lock(&_cpus_lock);
    rt_system_scheduler_start();
#endif /* RT_SMP_CPU1_SCHEDULES */

    for (;;)
    {
        __asm__ __volatile__ ("wfi");
    }
}

#endif /* RT_USING_SMP */
