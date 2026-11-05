/*
 * CPU1's tick.
 *
 * `rt_tick_increase()` (src/clock.c) does this on SMP:
 *
 *     rt_atomic_add(&(rt_cpu_self()->tick), 1);
 *     rt_sched_tick_increase(1);
 *     if (rt_cpu_get_id() != 0) return;      <- rt_timer_check() is CPU0 only
 *
 * so a core expected to schedule needs an interrupt of its own. Without one,
 * `rt_cpu_self()->tick` stays 0 (which `s31smp` reports), no time slice expires there, and
 * its idle thread can never be preempted.
 *
 * Two routes were tried and rejected before this one, both on measured grounds:
 *
 *   1. Call IDF's `vSystimerSetup()` from CPU1. It has a `cpuid != 0` branch, but its
 *      alarm-configuration loop is `for (cpuid = 0; cpuid < configNUM_CORES; ++cpuid)` and
 *      this build sets CONFIG_FREERTOS_UNICORE=y, so alarm 1 never gets a period or PERIOD
 *      mode - the branch only enables an interrupt that cannot fire. Measured: cpu1 tick
 *      stayed 0. Its handler also indexes `s_handled_systicks[configNUM_CORES]`, a
 *      one-element array here, with the core id.
 *
 *   2. Program the SYSTIMER alarm directly. `hal/systimer_hal.h` and `hal/systimer_ll.h`
 *      are exported by esp_hal_systimer through target-conditional INCLUDE_DIRS that this
 *      component's PRIV_REQUIRES does not surface - adding the dependency changed nothing
 *      (project_description.json still listed `freertos, riscv`).
 *
 * So it reuses what the port already has: `s31_timer_start()` in gptimer.c, a per-channel
 * TIMG timer with a callback, verified by `s31timer` on this board. `esp_intr_alloc()`
 * routes a line to the calling core, so starting it from CPU1 gives CPU1 the interrupt.
 */

#include <rtthread.h>

#include "sdkconfig.h"
#include "esp_attr.h"
#include "riscv/csr_clic.h"
#include "soc/interrupt_core0_reg.h"
#include "soc/interrupt_core1_reg.h"
#include "soc/clic_reg.h"

#ifdef RT_USING_SMP

/*
 * Everything below is CPU1 scheduling's platform half, so it lives behind the same
 * switch as the scheduler call it exists for. With the switch off there is no caller and
 * the checker is right to call it dead code - `s31smp` reports the parked core instead.
 */
#ifdef RT_SMP_CPU1_SCHEDULES

/* From port_timer.h: channel count and the callback type. */
#include "port_timer.h"

/*
 * Channel 3 of four. 0-2 are what `s31timer`, MCPWM and the RMT instrument use; 3 is
 * never started elsewhere in this image.
 */
#define S31_CPU1_TICK_TIMER 3

/*
 * Flash-operation cooperation, read by CPU1's tick handler and written by CPU0's flash
 * path. Both live in DRAM so they stay readable while the flash cache is off, and both
 * are declared here because the handler that reads them is further down.
 */
volatile unsigned s31_flash_op;
volatile unsigned s31_flash_ack;

/* Counts the ticks this core took, so `s31smp` can report a number instead of a claim. */
volatile unsigned s31_cpu1_ticks;

/* How many times the TIMG ISR body ran on CPU1. Distinct from the tick: if this stays 0
 * the handler was never entered, which is a different fault from "entered but the tick
 * did not advance". */
volatile unsigned s31_cpu1_isr_runs;

/* Ticks that arrived before this core had a thread (expected: a few, right after the
 * enable). Non-zero proves the ISR really is being entered. */
volatile unsigned s31_cpu1_ticks_early;

/*
 * CPU1's interrupt threshold, and why it has to be set here.
 *
 * IDF's xPortStartScheduler() runs on every core and begins with
 *
 *     esprv_int_set_threshold(RVHAL_INTR_ENABLE_THRESH);
 *     rv_utils_intr_global_enable();
 *
 * This port does the first of those once, from rt_hw_board_init(), which runs on CPU0.
 * `mintthresh` is a per-core CSR and the CLIC check is `int_ctl_byte > mintthresh_byte`,
 * so CPU1 keeps whatever reset left there - a threshold that masks everything. The
 * symptom is not a crash: CPU1 enters the scheduler and then never takes an interrupt, so
 * a thread bound to it never runs and its tick never fires.
 *
 * Both statements are per-core register writes, which is why they belong at the top of
 * CPU1's own path rather than in shared code.
 */
void rt_hw_secondary_cpu_irq_enable(void)
{
    extern void esprv_int_set_threshold(int priority_threshold);

    /*
     * FPU first, and it has to be first.
     *
     * The trap entry saves f0..f31 unconditionally when the toolchain has
     * __riscv_flen == 32 (vectors_rtthread.S:34/113), on the stated assumption that
     * mstatus.FS is non-zero in every context it can run on - which is true for every
     * thread, because rt_hw_stack_init_core() builds those frames with mstatus=0x7880.
     *
     * CPU1 is the exception: it starts from reset and takes its first trap from this
     * entry code, with FS=0. Measured:
     *
     *     CPU0 mstatus = 0x80006088   FS = 3
     *     CPU1 mstatus = 0x00000009   FS = 0
     *
     * so the first `fsw` on CPU1 is an illegal instruction and the core dies inside its
     * own trap handler. That is why the spin loop stops mid-way, why the TIMG ISR is
     * never entered, and why mintstatus.MIL still reads 0.
     *
     * FS=3 (dirty) rather than 1: the handler saves the registers, it does not merely
     * initialise them, so the state has to be one where the save is legal.
     */
    __asm__ volatile ("csrs mstatus, %0" ::
                      "r"((rt_uint32_t)((3u << 13) | (3u << 11))) : "memory");

    esprv_int_set_threshold(RVHAL_INTR_ENABLE_THRESH);

    /* Set mstatus.MIE directly rather than through rv_utils_intr_global_enable():
     * that one is FORCE_INLINE_ATTR, so declaring it extern here produces an undefined
     * reference instead of an inlined `csrrs` (measured). This is the same instruction
     * the trap path uses to re-enable interrupts for nesting. */
    __asm__ volatile ("csrsi mstatus, 8" ::: "memory");
}

static void s31_cpu1_tick_cb(void *arg)
{
    RT_UNUSED(arg);

    /* First statement, before anything can fault: this separates "the handler was
     * entered" from "the handler got as far as the tick". */
    s31_cpu1_isr_runs++;

    /*
     * If CPU0 has its flash cache off, acknowledge and get out.
     *
     * Everything below this point reaches flash: rt_tick_increase() is at 0x400116a4,
     * i.e. IROM. Fetching it while the cache is disabled is the cache error that killed
     * WiFi. The variables read here are in DRAM and this function is IRAM, so the check
     * itself is safe.
     */
    if (s31_flash_op)
    {
        s31_flash_ack = 1;
        return;
    }

    /*
     * Do not touch the kernel until this core has a thread.
     *
     * rt_tick_increase() calls rt_sched_tick_increase(), which reads
     * rt_thread_self()->... The tick is enabled before rt_system_scheduler_start(), so
     * the first tick arrives while pcpu->current_thread is still NULL - a NULL
     * dereference from an ISR on a core whose scheduler has not started, which stops the
     * core without reporting anything. Measured symptom: stage stops at 3 (right after
     * the tick was enabled) and no counter past it ever moves.
     */
    if (rt_thread_self() == RT_NULL)
    {
        s31_cpu1_ticks_early++;
        return;
    }

    s31_cpu1_ticks++;

    /* The same call CPU0's tick makes from xPortSysTickHandler(). It increments this
     * core's tick and expires this core's time slice; the timer list is CPU0's business
     * and rt_tick_increase() splits that itself. */
    rt_tick_increase();
}

rt_err_t rt_hw_secondary_cpu_tick_init(void)
{
    rt_err_t rc;
    extern volatile unsigned s31_cpu1_stage;
    extern volatile unsigned s31_cpu1_tick_started;

    rc = s31_timer_start(S31_CPU1_TICK_TIMER, RT_TICK_PER_SECOND,
                         s31_cpu1_tick_cb, RT_NULL);

    /* Past the timer creation and its esp_intr_alloc(). If the board stops before this
     * line is visible, that call is where it stops. */
    s31_cpu1_tick_started = 1u;
    s31_cpu1_stage = 5u;
    if (rc != RT_EOK)
    {
        rt_kprintf("smp: CPU1 tick timer %d failed (%d)\n",
                   S31_CPU1_TICK_TIMER, (int)rc);
    }

    /*
     * Enable the line - on THIS core's window.
     *
     * Measured, from CPU0 reading both CLIC windows:
     *
     *     line16 ie core0=1 core1=0   (ctrl 0x3fc10100 / 0x1fc00000)
     *
     * CPU0's control word has IE (bit 8); CPU1's is the reset value 0x1fc00000 - no IE,
     * priority field 0x1f, i.e. level 0 and masked. The source is routed to interrupt 0
     * on both cores and CPU1's trap vectors are correct, so this enable is the one thing
     * that was never done.
     *
     * `rt_hw_interrupt_umask()` writes CLIC_INT_CTRL_REG(inum + 16), and
     * DR_REG_CLIC_CTRL_BASE is the CURRENT core's window (soc/interrupt_reg.h), so
     * calling it here - on CPU1 - enables CPU1's line. That is also why calling it from
     * CPU0 could never have worked. INUM 0 is external line 16, which is what the TIMG
     * source maps to.
     *
     * `esp_intr_enable()` was tried first and did not change the result.
     */
    s31_cpu1_stage = 6u;

    rt_hw_interrupt_umask(0);

    /*
     * Vectored mode, on this core's window.
     *
     * Measured difference between the two cores' line-16 control words:
     *
     *     CPU0 = 0x3fc10100    SHV(bit16)=1
     *     CPU1 = 0x3fc00101    SHV(bit16)=0
     *
     * CLIC_INT_ATTR_SHV means "hardware vector interrupt". With it clear the hardware
     * jumps to `mtvec`, and `_vector_table` is a single `j _panic_handler` - so the tick
     * entered the panic path, which never returns and never reaches the TIMG ISR. That
     * matches every measurement: the spin loop stopped mid-way, isr_runs stayed 0, and
     * mintstatus.MIL was 0 (not inside a handler).
     *
     * mtvt[16] and mtvt[26] both hold _interrupt_handler, so the table was fine all
     * along; the line just was not vectored.
     *
     * CLIC_INT_CTRL_REG is the current core's window, so this write lands on CPU1.
     */
    {
        volatile rt_uint32_t *ctrl = (volatile rt_uint32_t *)
            (DR_REG_CLIC_CTRL_BASE + 16 * 4);

        *ctrl |= CLIC_INT_ATTR_SHV;
    }

    s31_cpu1_stage = 7u;

    return rc;
}

/*
 * Release CPU1 into the scheduler.
 *
 * INIT_APP_EXPORT, so it runs from main_thread_entry: by then rt_thread_idle_init() has
 * created both idle threads and CPU0's scheduler is running. Earlier - board stage or
 * component stage - and CPU1 would start with an empty ready table and trip
 * RT_ASSERT(to_thread) in _scheduler_get_highest_priority_thread(); measured as a
 * `rst:0x8 HP_SYS_HP_WDT1_RESET` loop.
 *
 * Deliberately not a semaphore: CPU1 polls this before it has a scheduler, so it cannot
 * block on an IPC object whose implementation assumes one.
 */
extern volatile unsigned s31_cpu1_go;


static int s31_cpu1_release(void)
{
    s31_cpu1_go = 1u;

    /*
     * Sample CPU1 from this core, before anything else can go wrong.
     *
     * The board resets (LP watchdog) within a second or two of CPU1 scheduling, so a
     * shell command never gets to read the counters. Printing them here is the only way
     * to tell "CPU1 takes no interrupt" from "CPU1 takes the tick and then breaks
     * something" - two findings with completely different fixes.
     */
    rt_thread_mdelay(300);
    {
        extern volatile unsigned s31_cpu1_ticks;
        extern volatile unsigned s31_cpu1_alive;
        struct rt_cpu *pcpu1 = rt_cpu_index(1);

        extern volatile unsigned s31_cpu1_entry_count;
        extern volatile unsigned s31_cpu1_mtvec;
        extern volatile unsigned s31_cpu1_mtvt;

        rt_kprintf("smp: after go, cpu1 alive=0x%x ticks=%u cur=%p idle=%p\n",
                   (unsigned)s31_cpu1_alive,
                   (unsigned)s31_cpu1_ticks,
                   (void *)pcpu1->current_thread,
                   (void *)pcpu1->idle_thread);

        /* What CPU1's own CSRs say. Both should equal the addresses smp_slave.c wrote;
         * anything else means the vector setup did not take effect on that core. */
        rt_kprintf("smp: cpu1 entries=%u mtvec=0x%08x mtvt=0x%08x\n",
                   (unsigned)s31_cpu1_entry_count,
                   (unsigned)s31_cpu1_mtvec,
                   (unsigned)s31_cpu1_mtvt);

        /* Same window read again, now that CPU1 has had 300 ms to run its own enable.
         * The report above runs first (INIT_APP_EXPORT order), before the go flag, so its
         * core1 value is the pre-state by construction. */
        {
            volatile rt_uint32_t *c0 = (volatile rt_uint32_t *)
                (DR_REG_CLIC_CTRL_BASE + 16 * 4);
            volatile rt_uint32_t *c1 = (volatile rt_uint32_t *)
                (DR_REG_CLIC_CTRL_BASE + DUALCORE_CLIC_CTRL_OFF + 16 * 4);

            rt_kprintf("smp: after go, line16 ie core0=%u core1=%u (0x%08x / 0x%08x)\n",
                       (unsigned)((*c0 >> 8) & 1u), (unsigned)((*c1 >> 8) & 1u),
                       (unsigned)*c0, (unsigned)*c1);

        {
            extern volatile unsigned s31_cpu1_mintthresh;
            extern volatile unsigned s31_cpu1_mstatus;

            rt_uint32_t t0v, tv0, ms0, mth0;

            __asm__ volatile ("csrr %0, mtvec" : "=r"(t0v));
            __asm__ volatile ("csrr %0, 0x307" : "=r"(tv0));
            __asm__ volatile ("csrr %0, mstatus" : "=r"(ms0));
            __asm__ volatile ("csrr %0, 0x347" : "=r"(mth0));

            rt_kprintf("smp: cpu1 mintthresh=0x%02x mstatus=0x%08x mie=%u\n",
                       (unsigned)s31_cpu1_mintthresh,
                       (unsigned)s31_cpu1_mstatus,
                       (unsigned)((s31_cpu1_mstatus >> 3) & 1u));

            /* Same four numbers from the core that DOES service this line. Anything
             * that differs here is the bug; anything equal rules itself out. */
            rt_kprintf("smp: cpu0 mtvec=0x%08x mtvt=0x%08x mintthresh=0x%02x "
                       "mstatus=0x%08x\n",
                       (unsigned)t0v, (unsigned)tv0, (unsigned)mth0, (unsigned)ms0);

            {
                extern volatile unsigned s31_cpu1_isr_runs;

                extern volatile unsigned s31_cpu1_spin;

                extern volatile unsigned s31_cpu1_stage;

                extern volatile unsigned s31_cpu1_tick_started;

                extern volatile unsigned s31_cpu1_ticks_early;

                rt_kprintf("smp: cpu1 stage=%u tick_started=%u isr_runs=%u ticks=%u "
                           "early=%u spin=%u\n",
                           (unsigned)s31_cpu1_stage,
                           (unsigned)s31_cpu1_tick_started,
                           (unsigned)s31_cpu1_isr_runs,
                           (unsigned)s31_cpu1_ticks,
                           (unsigned)s31_cpu1_ticks_early,
                           (unsigned)s31_cpu1_spin);
            }

            {
                extern volatile unsigned s31_cpu1_mintstatus;
                rt_uint32_t ms0;

                __asm__ volatile ("csrr %0, 0xfb1" : "=r"(ms0));

                rt_kprintf("smp: cpu1 mintstatus=0x%08x mil=0x%02x "
                           "(cpu0 0x%08x mil=0x%02x)\n",
                           (unsigned)s31_cpu1_mintstatus,
                           (unsigned)((s31_cpu1_mintstatus >> 24) & 0xff),
                           (unsigned)ms0, (unsigned)((ms0 >> 24) & 0xff));
            }
        }
        }
    }

    return 0;
}
INIT_APP_EXPORT(s31_cpu1_release);

#endif /* RT_SMP_CPU1_SCHEDULES */


/*
 * Flash-operation cooperation.
 *
 * `spi_flash_disable_interrupts_caches_and_other_cpu()` in this image has no cross-core
 * handshake - verified by disassembling it: op_lock, non-iram interrupt disable, cache
 * disable, return. IDF's dual-core half is compiled out because this build sets
 * CONFIG_FREERTOS_UNICORE / CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE, and
 * `esp_ipc_call_nonblocking` is absent from the ELF entirely.
 *
 * So CPU1 keeps running while CPU0's flash cache is off, and any CPU1 instruction fetch
 * from flash faults. Measured: MCAUSE 0x19 (cache error) at spimem_flash_ll_set_cs_pin,
 * reached from spiflash_start_core - and it only happens when CPU1's tick is running,
 * because the tick handler is the one thing CPU1 does that calls into flash
 * (`rt_tick_increase` at 0x400116a4; the handler itself is IRAM at 0x2f002d9c).
 *
 * The pair below gives the flash driver something to wait for. Both variables are in
 * DRAM, so they stay readable while the cache is off.
 */
/*
 * Called by CPU0 before it disables the flash cache. Returns only once CPU1 has
 * acknowledged, which means it is inside the IRAM-only part of its tick handler or in
 * wfi - either way it cannot fetch from flash until s31_flash_op is cleared.
 *
 * The timeout is not decoration: if CPU1 has no tick (RT_SMP_CPU1_NO_TICK) nothing will
 * ever acknowledge, and blocking forever would turn a missing feature into a hang. On
 * timeout the operation proceeds - the same behaviour as the single-core build this
 * replaces, which is what it did before the port enabled a second core.
 */
/* Strong definitions of the hooks spi_flash/cache_utils.c declares weak. */
void IRAM_ATTR spi_flash_other_core_stop(void)
{
    rt_uint32_t spins = 0;

    s31_flash_ack = 0;
    s31_flash_op = 1;

    /* CPU1's handler is short; a few thousand iterations is generous. */
    while (s31_flash_ack == 0 && spins < 200000u)
    {
        spins++;
    }
}

void IRAM_ATTR spi_flash_other_core_resume(void)
{
    s31_flash_op = 0;
}

/*
 * Print what CPU1's own CSRs say, whether or not it schedules.
 *
 * Separate from s31_cpu1_release() on purpose: that one only exists with
 * RT_SMP_CPU1_SCHEDULES on, and that configuration reset-loops. CPU1 runs its entry code
 * either way (s31_cpu1_alive proves it), so the vector-table question can be answered on
 * the stable build.
 *
 * INIT_APP_EXPORT runs from main_thread_entry, after main() has had a chance to run;
 * the delay gives CPU1 time to have executed its entry and stored the values.
 *
 * Not static, and named in a `-u` in CMakeLists.txt: nothing references it, so
 * without that anchor --gc-sections drops this whole object and its init-table
 * entry with it. Measured - the symbol was in the .obj and absent from the ELF.
 */
int s31_cpu1_report(void)
{
    extern volatile unsigned s31_cpu1_alive;
    extern volatile unsigned s31_cpu1_entry_count;
    extern volatile unsigned s31_cpu1_mtvec;
    extern volatile unsigned s31_cpu1_mtvt;

    rt_thread_mdelay(200);

    rt_kprintf("smp: cpu1 alive=0x%x entries=%u mtvec=0x%08x mtvt=0x%08x\n",
               (unsigned)s31_cpu1_alive,
               (unsigned)s31_cpu1_entry_count,
               (unsigned)s31_cpu1_mtvec,
               (unsigned)s31_cpu1_mtvt);

    /*
     * Where is the tick source routed?
     *
     * The vector table on CPU1 is correct (mtvec/mtvt match _vector_table/_mtvt_table),
     * so this is the remaining question. Each source has one MAP register per core, at
     * the same offset from each core's base, and the value is the interrupt number the
     * source is delivered as:
     *
     *   0 = delivered as interrupt 0 on that core
     *   6 = INT_MUX_DISABLED_INTNO, i.e. switched off there
     */
    {
        extern int s31_timer_irq_source(rt_uint32_t idx);
        int src = s31_timer_irq_source(1);   /* group 0, timer 1 = the CPU1 tick */

        rt_kprintf("smp: timg src=%d map core0=%d core1=%d\n", src,
                   (int)(*(volatile rt_uint32_t *)
                         (INTERRUPT_CORE0_TIMERGRP0_T1_INTR_MAP_REG)),
                   (int)(*(volatile rt_uint32_t *)
                         (INTERRUPT_CORE1_TIMERGRP0_T1_INTR_MAP_REG)));
    }

    /*
     * Is the line enabled in CPU1's CLIC window?
     *
     * The CLIC control block is per-core: DR_REG_CLIC_CTRL_BASE is the CURRENT core's
     * window and +DUALCORE_CLIC_CTRL_OFF is the other one (soc/interrupt_reg.h). So from
     * CPU0 both copies can be read, which turns "did the enable land on CPU1" into a
     * measurement instead of an assumption.
     *
     * Line 16 = external interrupt 0 = what source 26 is mapped to above.
     */
    {
        volatile rt_uint32_t *c0 = (volatile rt_uint32_t *)
            (DR_REG_CLIC_CTRL_BASE + 16 * 4);
        volatile rt_uint32_t *c1 = (volatile rt_uint32_t *)
            (DR_REG_CLIC_CTRL_BASE + DUALCORE_CLIC_CTRL_OFF + 16 * 4);

        rt_kprintf("smp: line16 ie core0=%u core1=%u (ctrl 0x%08x / 0x%08x)\n",
                   (unsigned)((*c0 >> 8) & 1u), (unsigned)((*c1 >> 8) & 1u),
                   (unsigned)*c0, (unsigned)*c1);
    }

    /*
     * What is in the CLIC vector slots?
     *
     * mtvec carries MODE=3, so the hardware jumps to `mtvt + 4 * (mcause - 16)`. With the
     * tick source mapped to interrupt 0, CPU1's trap for it lands on `_mtvt_table[0]`.
     *
     * This is a read of the live image, not of the source: if slot 0 holds
     * `_panic_handler` (which is what `_system_int_handler` expands to in vectors_clic.S),
     * then a tick on CPU1 enters the panic path - which does not return, and which would
     * explain "trap taken, ISR never entered, core stops" exactly.
     */
    {
        extern void _interrupt_handler(void);
        extern void _panic_handler(void);
        extern const void *_mtvt_table[48];

        rt_kprintf("smp: mtvt[0]=%p [16]=%p [26]=%p  ih=%p ph=%p\n",
                   _mtvt_table[0], _mtvt_table[16], _mtvt_table[26],
                   (void *)_interrupt_handler, (void *)_panic_handler);

        {
            extern void *rt_irq_stack_top[2];

            /* Just the table: the array itself is static to smp_istack.c, and the
             * question is only whether top[1] is above top[0] and distinct from it. */
            rt_kprintf("smp: istack top[0]=%p top[1]=%p\n",
                       rt_irq_stack_top[0], rt_irq_stack_top[1]);
        }
    }

    return 0;
}
INIT_APP_EXPORT(s31_cpu1_report);

#endif /* RT_USING_SMP */
