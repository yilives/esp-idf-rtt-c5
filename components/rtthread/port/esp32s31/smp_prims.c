/*
 * SMP hardware primitives for the S31, as a separate translation unit.
 *
 * These six functions are what the kernel's SMP scheduler expects from the CPU
 * port, and none of them exist yet in this port:
 *
 *   rt_hw_spin_lock_init / _lock / _unlock   (kernel critical sections)
 *   rt_hw_cpu_id                             (per-CPU indexing)
 *   rt_hw_secondary_cpu_up                   (starting core 1)
 *   rt_hw_secondary_cpu_idle_exec            (core 1 idle)
 *
 * They are written unconditionally but only *compiled into the link* when
 * RT_USING_SMP is on: the file is listed in the port's source set, and every
 * definition here is inside #ifdef RT_USING_SMP, so a single-core build sees an
 * empty object.
 *
 * ---------------------------------------------------------------------------
 * Why a ticket lock rather than a test-and-set.
 *
 * A test-and-set spinlock lets any waiter win, so under contention one core can
 * be starved indefinitely while another repeatedly re-acquires. The ticket lock
 * hands out numbers in arrival order (amoadd) and each waiter spins on its own
 * ticket, which makes progress FIFO and bounded.
 *
 * The layout is 16-bit owner + 16-bit next packed into one word, which lets the
 * release be a single halfword store - the store must not clobber a ticket that
 * another core is allocating concurrently, and touching only `owner` guarantees
 * that.
 *
 * This mirrors RT-Thread's own RISC-V implementation
 * (libcpu/risc-v/common/cpuport.c) rather than inventing a variant: a lock
 * whose memory ordering differs from the kernel's expectations is far harder to
 * debug than one that is merely copied.
 */

#include <rthw.h>
#include <rtthread.h>

#include "cpuport.h"

#ifdef RT_USING_SMP

/*
 * Read the hart id.
 *
 * mhartid is what the hardware reports and what IDF uses
 * (rv_utils.c: RV_READ_CSR(mhartid)), so the port agrees with every other
 * component about which core it is on.
 */
int rt_hw_cpu_id(void)
{
    rt_uint32_t id;

    __asm__ volatile ("csrr %0, mhartid" : "=r"(id));

    return (int)id;
}

void rt_hw_spin_lock_init(rt_hw_spinlock_t *lock)
{
    lock->slock = 0;
}

void rt_hw_spin_lock(rt_hw_spinlock_t *lock)
{
    unsigned long  tmp;
    unsigned short ticket;

    /*
     * amoadd.aqrl adds 1 to the NEXT counter and returns the previous word.
     *
     * The two counters share one 32-bit word: `owner` in the low half and `next`
     * in the high half. Adding (1 << 16) therefore advances `next`, and the
     * returned value's HIGH half is the ticket this core now owns - the number
     * that `next` held before the increment.
     *
     * Getting the halves the wrong way round produces a lock that appears to
     * work under no contention and then deadlocks or admits two holders at
     * once: measured reasoning, ticket = tmp & 0xFFFF always returns 0, because
     * `next` lives in the upper half.
     */
    __asm__ volatile ("amoadd.w.aqrl %0, %2, (%1)"
                      : "=&r"(tmp)
                      : "r"(&lock->slock), "r"(1UL << 16)
                      : "memory");

    ticket = (unsigned short)((tmp >> 16) & 0xFFFFu);

    /*
     * Spin until the owner counter reaches our ticket.
     *
     * The wait is emitted as the raw encoding of `pause` (0x0100000f) rather
     * than the mnemonic: it needs the Zihintpause extension, which this
     * toolchain's default -march does not include, and the assembler rejects the
     * name outright ("unrecognized opcode `pause'"). The encoding is the same
     * instruction, and a core that does not implement it treats it as a no-op -
     * so the spin stays correct either way.
     */
    while ((unsigned short)(lock->slock & 0xFFFFu) != ticket)
    {
        __asm__ volatile (".4byte 0x0100000f");
    }

    /* Keep the critical section's accesses after observing the release. */
    __asm__ volatile ("fence iorw, iorw" ::: "memory");
}

void rt_hw_spin_unlock(rt_hw_spinlock_t *lock)
{
    /*
     * Publish the critical section's writes before releasing, then advance only
     * the owner half.
     *
     * The halfword store is deliberate: a full-word read-modify-write would race
     * with a concurrent amoadd that is allocating the next ticket, and could
     * lose it.
     */
    __asm__ volatile ("fence iorw, iorw" ::: "memory");

    *(volatile unsigned short *)(&lock->slock) =
        (unsigned short)(*(volatile unsigned short *)(&lock->slock) + 1u);

    __asm__ volatile ("fence iorw, iorw" ::: "memory");
}

void rt_hw_secondary_cpu_idle_exec(void)
{
    __asm__ volatile ("wfi");
}

/*
 * Start the secondary core.
 *
 * A no-op unless RT_USING_SMP_SLAVE is set. Starting CPU1 needs a boot address, a
 * stack and an entry point, and any of them wrong hangs the board with no output;
 * the version in smp_slave.c is not yet trustworthy, so the default keeps this
 * core single. The kernel still initialises both per-CPU structures either way -
 * it just never gets a thread running on core 1.
 */
#if !defined(RT_USING_SMP_SLAVE)
void rt_hw_secondary_cpu_up(void)
{
}
#endif

#endif /* RT_USING_SMP */
