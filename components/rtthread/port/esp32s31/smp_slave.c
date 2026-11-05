/*
 * Bring up CPU1.
 *
 * IDF starts the secondary core in cpu_start.c:start_other_core(), which does
 * four things that all have to happen, in this order:
 *
 *     cpu_utility_ll_enable_clock_and_reset_app_cpu();
 *     cpu_utility_ll_enable_clock_and_reset_app_cpu_int_matrix();
 *     ets_set_appcpu_boot_addr((uint32_t)entry);
 *     esp_cpu_unstall(1);
 *
 * The port cannot reuse IDF's call_start_cpu1() - it ends in SYS_STARTUP_FN(),
 * which is the IDF init chain and expects the FreeRTOS scheduler. What CPU1 needs
 * here is the RT-Thread equivalent:
 *
 *     1. set gp (the global pointer) - any code compiled with medany/low can use
 *        it, and CPU1 starts with whatever the ROM left there
 *     2. point mtvec/mtvt at the same vectors CPU0 uses
 *     3. take the scheduler lock, so CPU1 does not touch scheduler state while
 *        CPU0 is still finishing its own startup
 *     4. rt_system_scheduler_start() - which on the MP scheduler picks this
 *        core's idle thread and switches to it
 *
 * That is the same shape RT-Thread's other SMP ports use (for example
 * bsp/k210/board/cpuport_smp.c: secondary_cpu_c_start()).
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "esp_cpu.h"
#include "esp_attr.h"
#include "esp_private/startup_internal.h"
#include "hal/cpu_utility_ll.h"

/*
 * Off by default: starting CPU1 currently breaks the shell (banner and prompt
 * print, then no command runs - no panic, no reset). The sequence below matches
 * IDF's start_other_core(), but something in it is wrong for this port and it has
 * to be debugged on its own rather than guessed at.
 *
 * Keeping the code here, behind a switch, means the next attempt starts from a
 * known shape instead of a blank file.
 */
#if defined(RT_USING_SMP) && defined(RT_USING_SMP_SLAVE)

/* Provided by IDF; the ROM entry points live in the esp32s31.rom.api.ld fragment. */
extern void ets_set_appcpu_boot_addr(unsigned int addr);

/*
 * CPU1's entry point.
 *
 * IRAM_ATTR because it runs while CPU0 may still be configuring flash, and the
 * cache state on CPU1 is whatever CPU0 left behind.
 */
IRAM_ATTR static void s31_secondary_cpu_entry(void)
{
    __asm__ __volatile__(
        ".option push\n"
        ".option norelax\n"
        "la gp, __global_pointer$\n"
        ".option pop\n");

    /*
     * The same trap vector CPU0 uses.
     *
     * Without this, an interrupt delivered on this core jumps through whatever
     * the ROM left in mtvec.
     */
    {
        extern int _vector_table;
        extern int _mtvt_table;

        esp_cpu_intr_set_ivt_addr(&_vector_table);
#if SOC_INT_CLIC_SUPPORTED
        esp_cpu_intr_set_mtvt_addr(&_mtvt_table);
#endif
    }

    /* Prove this code ran - the banner's "cpus=2" does not prove it. */
    {
        extern volatile unsigned s31_cpu1_alive;
        s31_cpu1_alive = 0x5A5A5A5Au;
    }

    /*
     * Read the trap-vector CSRs back - into globals, not to the console.
     *
     * `esp_cpu_intr_set_ivt_addr()` / `set_mtvt_addr()` are called just above, but that
     * they take effect on a secondary core is an assumption. Reading them back answers it
     * in two CSR reads.
     *
     * Printing them here does not work: measured, the capture contains `smp: cpu1` and
     * then CPU0's banner overwrites the rest, because both cores write UART0 with no
     * arbitration. (That CPU1 reaches the console at all is itself a useful fact - it
     * rules out "the secondary core has no console path".) So the values are stored and
     * printed by CPU0 afterwards.
     *
     * The counter is not decoration: if this entry runs more than once, CPU1 is being
     * reset, which would also explain a tick count stuck at zero.
     */
    {
        extern volatile unsigned s31_cpu1_entry_count;
        extern volatile unsigned s31_cpu1_mtvec;
        extern volatile unsigned s31_cpu1_mtvt;

        rt_uint32_t v;

        s31_cpu1_entry_count++;

        __asm__ volatile ("csrr %0, mtvec" : "=r"(v));
        s31_cpu1_mtvec = v;

        __asm__ volatile ("csrr %0, 0x307" : "=r"(v));   /* mtvt */
        s31_cpu1_mtvt = v;
    }

#ifdef RT_USING_SMP_SLAVE_LOCK

    /*
     * Variant B: take the scheduler lock and stop.
     *
     * The point of this variant is the wait, not what happens after it:
     * rt_system_scheduler_start() releases _cpus_lock unconditionally, and CPU0
     * holds it from rtthread_startup() until its own call. So CPU1 acquiring it
     * proves the two cores are sequenced - CPU1 cannot get past this line until
     * CPU0 is done with its startup. See smp_slave_sched.c.
     */
    {
        extern void rt_hw_secondary_cpu_start_scheduler(void);
        rt_hw_secondary_cpu_start_scheduler();
    }

#else

    /*
     * Variant A (default): the core is up, parked, and stays out of the way.
     *
     * Verified: s31stat reports "cpu1: started (0x5a5a5a5a)", the shell answers,
     * s31test is 20/20 and it is stable across runs. What this proves is the part
     * that was genuinely unknown - CPU1 can be released, can execute IRAM code and
     * can see the shared vectors.
     */
    for (;;)
    {
        __asm__ __volatile__ ("wfi");
    }

#endif
}

/* Set by CPU1's entry; read by s31stat so the shell can report it. */
volatile unsigned s31_cpu1_alive;

/* How many times CPU1's entry has run. >1 means this core is being restarted. */
volatile unsigned s31_cpu1_entry_count;

/* What CPU1's trap-vector CSRs actually read back. Printed by CPU0. */
volatile unsigned s31_cpu1_mtvec;
volatile unsigned s31_cpu1_mtvt;

/* CPU1's interrupt gates, read on that core just before it starts scheduling. */
volatile unsigned s31_cpu1_mintthresh;
volatile unsigned s31_cpu1_mstatus;
volatile unsigned s31_cpu1_mintstatus;

/* Bumped by CPU1 in a plain loop before it starts scheduling. */
volatile unsigned s31_cpu1_spin;

/* How far CPU1 got before it stopped running: 1 = readbacks done, 2 = MIE set,
 * 3 = tick configured, 4 = spin loop done. A value that stops advancing names the
 * instruction after which this core never ran again. */
volatile unsigned s31_cpu1_stage;

/* Set once s31_timer_start() returned on CPU1. */
volatile unsigned s31_cpu1_tick_started;

void rt_hw_secondary_cpu_up(void)
{
    /*
     * Install the crosscore line first.
     *
     * Once CPU1 schedules it will be woken by RT_SCHEDULE_IPI, and an assert that no
     * handler is installed for a line that fires is a trap, not a warning. Doing it here
     * - before the release below - means CPU1 never runs with the line unhandled.
     */
    extern rt_err_t rt_hw_ipi_handler_install(int vector, void *handler);
    if (rt_hw_ipi_handler_install(0, RT_NULL) != RT_EOK)
    {
        rt_kprintf("smp: crosscore line not installed, CPU1 will not be woken\n");
    }

    /* Clock + reset release for the APP core. */
    cpu_utility_ll_enable_clock_and_reset_app_cpu();
    cpu_utility_ll_enable_clock_and_reset_app_cpu_int_matrix();

    ets_set_appcpu_boot_addr((unsigned int)(uintptr_t)s31_secondary_cpu_entry);

    esp_cpu_unstall(1);
}

#endif /* RT_USING_SMP && RT_USING_SMP_SLAVE */
