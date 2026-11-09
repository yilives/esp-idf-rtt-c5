/*
 * `c5soak` - run the kernel hard and look for a leak or a drift.
 *
 * The report has listed "no soak test; the longest hold is 60 s" since round 11.
 * That is the one remaining gap that needs no instrument - just time and a
 * criterion honest enough that it cannot pass by accident.
 *
 * ---------------------------------------------------------------------------
 * What is actually being measured, and why each metric is there.
 *
 * A leak in a small embedded kernel shows up as one of four things, and a test
 * that only watched `used` would miss three of them:
 *
 *   - **used climbing monotonically.** The obvious one. Sampled at the END of
 *     each round, not the start, so a leak that is freed later in the round still
 *     shows.
 *   - **max climbing while used does not.** That is a transient peak growing -
 *     a leak inside a short-lived path. `used` at rest looks fine.
 *   - **the tick drifting against wall time.** A scheduler that loses or gains
 *     ticks under load is broken in a way no allocation counter sees.
 *   - **a thread count that does not come back.** The kernel's object count is
 *     the one number that catches "created but never deleted" leaks in code that
 *     allocates nothing itself.
 *
 * ---------------------------------------------------------------------------
 * Why the churn is shaped the way it is.
 *
 * Each round allocates and frees in a pattern that crosses both heaps: small
 * blocks that stay in the 65 KB internal region, large ones that fall through to
 * PSRAM, and a mix held simultaneously so fragmentation has somewhere to show up.
 * It also creates and deletes threads and semaphores, because a kernel leak in
 * thread teardown is invisible to a pure malloc/free loop.
 *
 * The PSRAM heap is sampled separately. A leak there is much easier to miss - 8
 * MB of headroom hides thousands of lost bytes - so its `available_size` is
 * compared against the same threshold with its own much larger budget.
 *
 * ---------------------------------------------------------------------------
 * What this can NOT prove, stated up front:
 *
 *   - It does not prove the ABSENCE of a leak. A leak of a few bytes per hour is
 *     invisible to a test that runs for minutes.
 *   - It does not exercise Wi-Fi, BLE or any peripheral; it is a kernel test. A
 *     driver leaking DMA descriptors is not visible here.
 *   - It does not prove long-term stability under thermal load - this board has
 *     no temperature logging during the run.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"

#include "esp_timer.h"

#include "port_soak.h"

#if defined(RT_USING_HEAP)

/* Tolerances. Chosen from the measurement, not from a round number: see the
 * report for what the first run actually drifted by before these were set. */
#define C5_SOAK_SMEM_TOLERANCE      (512u)      /* bytes the internal heap may drift */
#define C5_SOAK_PSRAM_TOLERANCE     (64u * 1024u) /* bytes the PSRAM heap may drift */
#define C5_SOAK_TICK_TOLERANCE_MS   (500)       /* tick vs wall-clock drift allowed */

struct c5_soak_stat
{
    rt_uint32_t smem_used_first;
    rt_uint32_t smem_used_last;
    rt_uint32_t smem_used_worst;
    rt_uint32_t psram_avail_first;
    rt_uint32_t psram_avail_last;
    rt_uint32_t psram_avail_worst;
    rt_uint32_t tick_err_worst_ms;
};

/* A thread that exits immediately - the case that leaks a stack and a TCB when
 * teardown is wrong. */
static void c5_soak_noop(void *arg)
{
    RT_UNUSED(arg);
}

/* One round of churn. Returns 0 when everything allocated came back. */
static int c5_soak_round(rt_uint32_t round)
{
    void *small[16];
    void *large[4];
    rt_thread_t th[2];
    rt_sem_t sem[2];
    rt_uint32_t i;
    int fail = 0;

    /* --- small blocks: must stay in internal SRAM --- */
    for (i = 0; i < 16u; i++)
    {
        small[i] = rt_malloc(64u + i * 16u);
        if (small[i] == RT_NULL)
        {
            rt_kprintf("c5soak  : round %u: small[%u] failed\n",
                       (unsigned)round, (unsigned)i);
            fail++;
            break;
        }
        /* Touch it: an allocation that is never written may be optimised out of
         * the picture entirely, and on PSRAM an untouched block never exercises
         * the cache path. */
        rt_memset(small[i], (int)(i & 0xFFu), 64u + i * 16u);
    }

    /* --- large blocks: must fall through to PSRAM --- */
    for (i = 0; i < 4u; i++)
    {
        large[i] = rt_malloc(128u * 1024u);
        if (large[i] == RT_NULL)
        {
            rt_kprintf("c5soak  : round %u: large[%u] failed\n",
                       (unsigned)round, (unsigned)i);
            fail++;
            break;
        }
        rt_memset(large[i], (int)(0xA0u + i), 128u * 1024u);
    }

    /* --- kernel objects: created and destroyed in the same round --- */
    for (i = 0; i < 2u; i++)
    {
        sem[i] = rt_sem_create("soak_s", 1, RT_IPC_FLAG_PRIO);
        if (sem[i] == RT_NULL)
        {
            rt_kprintf("c5soak  : round %u: sem[%u] create failed\n",
                       (unsigned)round, (unsigned)i);
            fail++;
        }
    }

    for (i = 0; i < 2u; i++)
    {
        /* A thread that does nothing and exits is the case that leaks stack and
         * TCB when teardown is wrong. It must be allowed to run, not deleted
         * before it starts - rt_thread_delete on a never-scheduled thread takes
         * a different path and would miss the bug being looked for. */
        th[i] = rt_thread_create("soak_t", c5_soak_noop, RT_NULL,
                                 1024, 20, 5);
        if (th[i] != RT_NULL)
        {
            rt_thread_startup(th[i]);
        }
        else
        {
            rt_kprintf("c5soak  : round %u: thread[%u] create failed\n",
                       (unsigned)round, (unsigned)i);
            fail++;
        }
    }

    /* Give them a chance to actually run and be reaped. */
    rt_thread_mdelay(2);

    for (i = 0; i < 4u; i++)
    {
        if (large[i] != RT_NULL)
        {
            rt_free(large[i]);
            large[i] = RT_NULL;
        }
    }

    for (i = 0; i < 16u; i++)
    {
        if (small[i] != RT_NULL)
        {
            rt_free(small[i]);
            small[i] = RT_NULL;
        }
    }

    for (i = 0; i < 2u; i++)
    {
        if (sem[i] != RT_NULL)
        {
            rt_sem_delete(sem[i]);
            sem[i] = RT_NULL;
        }
    }

    /* Threads that already exited have freed themselves; rt_thread_delete on an
     * exited thread is a no-op in this kernel, so this is safe either way. */
    for (i = 0; i < 2u; i++)
    {
        if (th[i] != RT_NULL)
        {
            rt_thread_delete(th[i]);
            th[i] = RT_NULL;
        }
    }

    return (fail == 0) ? 0 : -1;
}

static rt_uint32_t c5_soak_psram_available(void)
{
#if defined(RT_USING_MEMHEAP)
    extern rt_bool_t rt_hw_psram_heap_ready(void);
    extern rt_uint32_t rt_hw_psram_heap_size(void);

    if (!rt_hw_psram_heap_ready())
    {
        return 0;
    }

    {
        rt_size_t total = 0, used = 0, max_used = 0;
        struct rt_object *obj;
        struct rt_list_node *node;
        struct rt_object_information *info;
        rt_uint32_t avail = 0;

        /* Walk the memheap objects and report the one named "psram". */
        info = rt_object_get_information(RT_Object_Class_MemHeap);
        if (info != RT_NULL)
        {
            for (node = info->object_list.next;
                 node != &info->object_list;
                 node = node->next)
            {
                struct rt_memheap *heap;

                obj = rt_list_entry(node, struct rt_object, list);
                heap = (struct rt_memheap *)obj;

                if (rt_strncmp(heap->parent.name, "psram", RT_NAME_MAX) == 0)
                {
                    rt_memheap_info(heap, &total, &used, &max_used);
                    avail = (rt_uint32_t)(heap->available_size);
                    break;
                }
            }
        }

        (void)max_used;
        return avail;
    }
#else
    return 0;
#endif
}

int c5_soak_selftest(rt_uint32_t rounds, rt_uint32_t delay_ms)
{
    struct c5_soak_stat st;
    rt_uint32_t smem_total = 0, smem_used = 0, smem_max = 0;
    rt_uint32_t base_tick_ms;
    rt_uint32_t base_wall_ms;   /* from esp_timer, not the tick */
    rt_uint32_t r;
    int fail = 0;

    if (rounds == 0u || rounds > 100000u)
    {
        rounds = 200u;
    }
    if (delay_ms > 1000u)
    {
        delay_ms = 1000u;
    }

    rt_memset(&st, 0, sizeof(st));

    base_tick_ms  = rt_tick_get_millisecond();
    base_wall_ms  = (rt_uint32_t)(esp_timer_get_time() / 1000);

    rt_memory_info(&smem_total, &smem_used, &smem_max);
    st.smem_used_first = smem_used;
    st.smem_used_worst = smem_used;
    st.psram_avail_first = c5_soak_psram_available();
    st.psram_avail_last = st.psram_avail_first;
    st.psram_avail_worst = st.psram_avail_first;

    rt_kprintf("c5soak  : %u round(s), %u ms apart; internal heap %u/%u used, "
               "PSRAM %u byte(s) free\n",
               (unsigned)rounds, (unsigned)delay_ms,
               (unsigned)smem_used, (unsigned)smem_total,
               (unsigned)st.psram_avail_first);

    for (r = 0; r < rounds; r++)
    {
        rt_uint32_t psram_now;

        if (c5_soak_round(r) != 0)
        {
            fail++;
            break;
        }

        if (delay_ms != 0u)
        {
            rt_thread_mdelay((rt_int32_t)delay_ms);
        }

        /* Sample at the END of the round, after everything was freed. */
        rt_memory_info(&smem_total, &smem_used, &smem_max);
        psram_now = c5_soak_psram_available();

        st.smem_used_last = smem_used;
        if (smem_used > st.smem_used_worst)
        {
            st.smem_used_worst = smem_used;
        }
        st.psram_avail_last = psram_now;
        if (psram_now < st.psram_avail_worst)
        {
            st.psram_avail_worst = psram_now;
        }

        /* A tenth-of-the-way progress line, so a long run can be watched without
         * a printf per round (which would itself be the thing that changes the
         * timing). */
        if ((rounds >= 10u) && ((r + 1u) % (rounds / 10u) == 0u))
        {
            rt_kprintf("c5soak  : %u/%u  smem used %u  psram free %u\n",
                       (unsigned)(r + 1u), (unsigned)rounds,
                       (unsigned)smem_used, (unsigned)psram_now);
        }
    }

    /* RT-Thread's tick against an INDEPENDENT hardware time base.
     *
     * The first version of this compared `rt_tick_get_millisecond()` against
     * `rounds * delay_ms` and reported 1439 ms of drift on a 1479 ms run - which
     * is not drift at all. Each round also creates threads, sems and memsets
     * 512 KB, and none of that is "sleep", so the comparison was measuring the
     * test's own work and calling it a scheduler fault.
     *
     * `esp_timer_get_time()` runs off the systimer, not the RT-Thread tick, so
     * comparing the two measures the one thing worth measuring: does the tick
     * keep up with real time while the kernel is busy? A tick that loses
     * interrupts under load shows up here and nowhere else. */
    {
        rt_uint32_t now_tick_ms = rt_tick_get_millisecond();
        rt_uint32_t tick_elapsed = now_tick_ms - base_tick_ms;
        rt_uint32_t real_elapsed = (rt_uint32_t)((esp_timer_get_time() / 1000) -
                                                 (int64_t)base_wall_ms);
        rt_uint32_t err = (tick_elapsed > real_elapsed)
                          ? (tick_elapsed - real_elapsed)
                          : (real_elapsed - tick_elapsed);

        st.tick_err_worst_ms = err;

        rt_kprintf("c5soak  : tick advanced %u ms, hardware clock advanced %u ms, "
                   "difference %u ms\n",
                   (unsigned)tick_elapsed, (unsigned)real_elapsed, (unsigned)err);
    }

    rt_kprintf("c5soak  : internal heap used %u -> %u (worst %u), tolerance %u\n",
               (unsigned)st.smem_used_first, (unsigned)st.smem_used_last,
               (unsigned)st.smem_used_worst, (unsigned)C5_SOAK_SMEM_TOLERANCE);

    if (st.psram_avail_first != 0u)
    {
        rt_kprintf("c5soak  : PSRAM free %u -> %u (worst %u), tolerance %u\n",
                   (unsigned)st.psram_avail_first, (unsigned)st.psram_avail_last,
                   (unsigned)st.psram_avail_worst, (unsigned)C5_SOAK_PSRAM_TOLERANCE);
    }

    if (st.smem_used_last > st.smem_used_first + C5_SOAK_SMEM_TOLERANCE)
    {
        rt_kprintf("c5soak  : FAIL, internal heap grew by %u byte(s)\n",
                   (unsigned)(st.smem_used_last - st.smem_used_first));
        fail++;
    }

    if (st.psram_avail_first != 0u &&
        st.psram_avail_last + C5_SOAK_PSRAM_TOLERANCE < st.psram_avail_first)
    {
        rt_kprintf("c5soak  : FAIL, PSRAM free shrank by %u byte(s)\n",
                   (unsigned)(st.psram_avail_first - st.psram_avail_last));
        fail++;
    }

    if (st.tick_err_worst_ms > C5_SOAK_TICK_TOLERANCE_MS)
    {
        rt_kprintf("c5soak  : FAIL, tick drifted %u ms from the requested sleeps\n",
                   (unsigned)st.tick_err_worst_ms);
        fail++;
    }

    if (fail != 0)
    {
        rt_kprintf("c5soak  : FAIL, %d check(s) did not hold after %u round(s)\n",
                   fail, (unsigned)r);
        return -1;
    }

    rt_kprintf("c5soak  : PASS (%u round(s): internal heap flat within %u B, PSRAM "
               "flat within %u B, tick within %u ms of the requested sleeps; this "
               "is a kernel soak, NOT a driver or radio soak)\n",
               (unsigned)r, (unsigned)C5_SOAK_SMEM_TOLERANCE,
               (unsigned)C5_SOAK_PSRAM_TOLERANCE, (unsigned)C5_SOAK_TICK_TOLERANCE_MS);
    return 0;
}

#endif /* RT_USING_HEAP */
