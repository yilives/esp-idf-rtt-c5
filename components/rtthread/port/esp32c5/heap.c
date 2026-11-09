/*
 * `c5heap` - prove which heap `rt_malloc()` actually used.
 *
 * The PSRAM heap is registered (round 33), but "registered" and "used" are
 * different claims and only one of them is testable from a configuration file.
 * This allocates across a range of sizes and reports, for each one, which pool
 * the pointer came from - measured by its address, not inferred from a textbook
 * expectation of how the fallback works.
 *
 * ---------------------------------------------------------------------------
 * What is being tested, and why the sizes are what they are.
 *
 * `_memheap_alloc()` tries the system heap first and only then walks the other
 * memheaps (memheap.c:774-809). The system heap is 65 KB, so:
 *
 *   - a SMALL request must come from internal SRAM, because the system heap
 *     succeeds on the first try;
 *   - a request LARGER than the free system heap must come from `psram`,
 *     because the fallback is the only thing left.
 *
 * Both directions are asserted. A test that only checked "a big allocation
 * works" would pass even if every allocation went to PSRAM, which would be the
 * wrong outcome - small objects in PSRAM pay the SPI-cache penalty on every
 * access.
 *
 * ---------------------------------------------------------------------------
 * Why the contents are written and read back.
 *
 * A pointer into the PSRAM region that is not actually backed by the part reads
 * back whatever the bus returns, and an allocator bug that hands out an
 * overlapping block reads back the other block's data. A position-dependent
 * pattern catches both: no two offsets hold the same byte, so a short block, an
 * offset error or an overlap all show up as a mismatch rather than as a
 * plausible run of zeros.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"

#if defined(SOC_SPIRAM_SUPPORTED)

#include "port_psram.h"

/* Where the PSRAM lives in the address space. `soc.h` defines both ends; a
 * pointer inside them came from the PSRAM heap. */
#define C5_IS_PSRAM_PTR(p)                                              \
    (((rt_ubase_t)(p) >= (rt_ubase_t)SOC_EXTRAM_DATA_LOW) &&            \
     ((rt_ubase_t)(p) <  (rt_ubase_t)SOC_EXTRAM_DATA_HIGH))

/* One allocation, checked. Returns 0 on success. */
static int c5_heap_probe(rt_uint32_t size, rt_bool_t expect_psram)
{
    rt_uint8_t *p;
    rt_uint32_t i;
    rt_uint32_t bad = 0;
    rt_bool_t in_psram;

    p = (rt_uint8_t *)rt_malloc(size);

    if (p == RT_NULL)
    {
        rt_kprintf("c5heap  : %7u B  FAILED to allocate\n", (unsigned)size);
        return -1;
    }

    in_psram = C5_IS_PSRAM_PTR(p) ? RT_TRUE : RT_FALSE;

    /* Position-dependent pattern: every byte differs from every neighbour, so
     * an overlapping block or a shifted copy cannot pass. */
    for (i = 0; i < size; i++)
    {
        p[i] = (rt_uint8_t)((i * 31u) ^ (i >> 8) ^ 0x5Au);
    }

    for (i = 0; i < size; i++)
    {
        rt_uint8_t want = (rt_uint8_t)((i * 31u) ^ (i >> 8) ^ 0x5Au);

        if (p[i] != want)
        {
            if (bad < 3u)
            {
                rt_kprintf("c5heap  :   +%u is 0x%02x, wrote 0x%02x\n",
                           (unsigned)i, p[i], want);
            }
            bad++;
        }
    }

    rt_kprintf("c5heap  : %7u B  at 0x%08x  %-7s  %s%s\n",
               (unsigned)size, (unsigned)(rt_ubase_t)p,
               in_psram ? "PSRAM" : "SRAM",
               (bad == 0u) ? "content OK" : "CONTENT MISMATCH",
               (in_psram == expect_psram) ? ""
                                          : "   <- wrong pool");

    rt_free(p);

    if (bad != 0u)
    {
        return -1;
    }

    if (in_psram != expect_psram)
    {
        return -1;
    }

    return 0;
}

/* `c5heap` - allocation routing across four orders of magnitude. */
int c5_heap_selftest(void)
{
    rt_size_t total = 0, used = 0, max_used = 0;
    int fail = 0;

    rt_kprintf("c5heap  : PSRAM heap %s, %u byte(s); window 0x%08x..0x%08x\n",
               rt_hw_psram_heap_ready() ? "registered" : "NOT registered",
               (unsigned)rt_hw_psram_heap_size(),
               (unsigned)SOC_EXTRAM_DATA_LOW, (unsigned)SOC_EXTRAM_DATA_HIGH);

    if (!rt_hw_psram_heap_ready())
    {
        rt_kprintf("c5heap  : FAIL, no PSRAM heap - rt_malloc() can only reach the "
                   "65 KB internal region\n");
        return -1;
    }

    rt_memory_info(&total, &used, &max_used);
    rt_kprintf("c5heap  : system heap before: total=%u used=%u max=%u\n",
               (unsigned)total, (unsigned)used, (unsigned)max_used);

    /* Small: must come from internal SRAM, because the system heap answers first.
     * 1 KB and 16 KB are both comfortably under the 48 KB that is normally free. */
    fail += (c5_heap_probe(1024u, RT_FALSE) != 0) ? 1 : 0;
    fail += (c5_heap_probe(16u * 1024u, RT_FALSE) != 0) ? 1 : 0;

    /* Large: cannot fit in the system heap, so the fallback must reach PSRAM.
     * 1 MB and 4 MB are both far beyond 65 KB. */
    fail += (c5_heap_probe(1024u * 1024u, RT_TRUE) != 0) ? 1 : 0;
    fail += (c5_heap_probe(4u * 1024u * 1024u, RT_TRUE) != 0) ? 1 : 0;

    /* Hold several at once, to prove the PSRAM heap can serve more than one
     * outstanding block rather than only the first. */
    {
        void *blocks[8];
        int n = 0;
        int i;

        for (i = 0; i < 8; i++)
        {
            blocks[i] = rt_malloc(512u * 1024u);
            if (blocks[i] == RT_NULL)
            {
                break;
            }
            n++;
        }

        rt_kprintf("c5heap  : %d x 512 KB held simultaneously\n", n);

        for (i = 0; i < n; i++)
        {
            rt_free(blocks[i]);
        }

        if (n < 8)
        {
            /* 8 x 512 KB = 4 MB, well inside an 8 MB pool. Fewer means the
             * fallback only serves one block at a time. */
            rt_kprintf("c5heap  : FAIL, only %d of 8 blocks could be held\n", n);
            fail++;
        }
    }

    rt_memory_info(&total, &used, &max_used);
    rt_kprintf("c5heap  : system heap after: total=%u used=%u max=%u\n",
               (unsigned)total, (unsigned)used, (unsigned)max_used);

    if (fail != 0)
    {
        rt_kprintf("c5heap  : FAIL, %d check(s) did not hold\n", fail);
        return -1;
    }

    rt_kprintf("c5heap  : PASS (small allocations stay in SRAM, large ones land in "
               "PSRAM, contents survive, and 4 MB can be held at once)\n");
    return 0;
}

#endif /* SOC_SPIRAM_SUPPORTED */
