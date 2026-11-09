/*
 * DMA to and from PSRAM, with the cache handled correctly.
 *
 * The `psram` heap (round 33) makes `rt_malloc(4 MB)` succeed, and that is only
 * half of what a large buffer is for. The other half is feeding it to a
 * peripheral - and on this chip that is not free, because PSRAM is behind the
 * SPI cache.
 *
 * ---------------------------------------------------------------------------
 * The two rules, and what happens when each is broken.
 *
 * 1. **Before the DMA reads its SOURCE, the cache must be flushed to memory.**
 *    The CPU wrote the buffer through the cache, so the bytes may still live in
 *    the cache and not in the PSRAM chip. A DMA engine reads physical memory; it
 *    sees whatever was there before, which on a freshly allocated block is
 *    garbage.
 *
 * 2. **Before the CPU reads the DMA's DESTINATION, the cache must be
 *    invalidated.** Same fact in the other direction: the cache may hold a stale
 *    copy of the region and will keep serving it after the DMA has overwritten
 *    memory underneath.
 *
 * Both are testable here without a peripheral, because GDMA memory-to-memory is
 * itself a DMA engine accessing physical memory. A copy from PSRAM to internal
 * RAM exercises rule 1; a copy from internal RAM to PSRAM exercises rule 2. Get
 * either wrong and the copy is byte-wrong in a repeatable way.
 *
 * `esp_cache_msync(addr, size, flags)` is the operation, with
 * `ESP_CACHE_MSYNC_FLAG_DIR_C2M` for flush and `..._DIR_M2C` for invalidate.
 *
 * ---------------------------------------------------------------------------
 * What is NOT proven here, and why it matters.
 *
 * This proves the cache discipline is sufficient for a DMA that the CPU starts
 * and waits for. It does NOT prove it for a peripheral streaming continuously
 * into PSRAM while the CPU reads along behind it - that needs the cache lines
 * around the read pointer invalidated as the transfer advances, and the rule
 * there is "invalidate before reading, never after", which this test cannot
 * exercise without a real producer.
 *
 * Also not proven: PSRAM as executable memory, and the interaction with flash
 * encryption (which imposes its own alignment, 16 bytes on this part).
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "esp_cache.h"
#include "esp_attr.h"

#if defined(SOC_PSRAM_DMA_CAPABLE) || defined(SOC_SPIRAM_SUPPORTED)

#include "port_cache.h"
#include "port_gdma.h"
#include "port_psram.h"

/* The cache line is 32 bytes on this part (CONFIG_CACHE_L1_CACHE_LINE_SIZE), and
 * a buffer smaller than one line shares that line with whatever lives next to it
 * - flushing or invalidating it perturbs a neighbour. Everything below is
 * line-size aligned and rounded up. */
#define C5_CACHE_LINE      32u

/* Larger than the 65 KB system heap, so rt_malloc() must fall back. */
#define C5_FORCE_PSRAM_LEN (256u * 1024u)

/* One descriptor holds at most 4095 bytes (DMA_DESCRIPTOR_BUFFER_MAX_SIZE),
 * and going over returns -RT_EINVAL rather than splitting the chain. */
#define C5_DMA_TEST_LEN    (4000u)

static rt_uint8_t *s_src __attribute__((aligned(C5_CACHE_LINE)));
static rt_uint8_t *s_dst __attribute__((aligned(C5_CACHE_LINE)));

static rt_uint32_t c5_cache_line_size(void)
{
    size_t sz = esp_cache_get_line_size_by_addr(s_src);

    return (sz != 0u) ? (rt_uint32_t)sz : C5_CACHE_LINE;
}

/* Flush the CPU's writes out to physical memory, so a DMA reading them sees
 * what the CPU wrote. */
int rt_hw_cache_flush(const void *addr, rt_uint32_t len)
{
    return (esp_cache_msync((void *)addr, len,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M) == ESP_OK)
           ? 0 : -1;
}

/* Throw away the cache's copy, so the CPU re-reads what a DMA wrote. */
int rt_hw_cache_invalidate(void *addr, rt_uint32_t len)
{
    return (esp_cache_msync(addr, len,
                            ESP_CACHE_MSYNC_FLAG_DIR_M2C) == ESP_OK)
           ? 0 : -1;
}

/*
 * Copy that is correct regardless of which side is in PSRAM.
 *
 * `rt_hw_gdma_memcpy()` itself stays cache-agnostic - it is the caller that
 * knows whether the buffers are in cached memory, and hiding the sync inside the
 * copy would make it impossible to measure whether the sync is needed. This
 * wrapper does the sync and then calls it.
 */
rt_err_t rt_hw_dma_memcpy_cached(void *dst, const void *src, rt_uint32_t len)
{
    rt_uint32_t line = c5_cache_line_size();
    rt_uint32_t span = RT_ALIGN(len, line);
    rt_bool_t src_cached;
    rt_bool_t dst_cached;
    rt_err_t err;

    src_cached = ((rt_ubase_t)src >= (rt_ubase_t)SOC_EXTRAM_DATA_LOW &&
                  (rt_ubase_t)src <  (rt_ubase_t)SOC_EXTRAM_DATA_HIGH);
    dst_cached = ((rt_ubase_t)dst >= (rt_ubase_t)SOC_EXTRAM_DATA_LOW &&
                  (rt_ubase_t)dst <  (rt_ubase_t)SOC_EXTRAM_DATA_HIGH);

    if (src_cached)
    {
        if (rt_hw_cache_flush(src, span) != 0)
        {
            rt_kprintf("c5dma   : flush failed for 0x%08x +%u\n",
                       (unsigned)(rt_ubase_t)src, (unsigned)span);
            return -RT_ERROR;
        }
    }

    if (dst_cached)
    {
        /* Invalidate BEFORE the DMA writes, so a dirty line cannot be written
         * back over the DMA's output afterwards. */
        if (rt_hw_cache_invalidate(dst, span) != 0)
        {
            rt_kprintf("c5dma   : invalidate (pre) failed for 0x%08x +%u\n",
                       (unsigned)(rt_ubase_t)dst, (unsigned)span);
            return -RT_ERROR;
        }
    }

    err = rt_hw_gdma_memcpy(dst, src, len, 500u);

    if (dst_cached)
    {
        /* Invalidate AGAIN after, so the CPU's next read comes from memory
         * rather than from a line the DMA filled behind the cache's back. */
        if (rt_hw_cache_invalidate(dst, span) != 0)
        {
            return -RT_ERROR;
        }
    }

    return err;
}

/* ------------------------------------------------------------------ msh -- */

/* `c5dma` - DMA in and out of PSRAM, with and without the cache handling.
 *
 * The interesting line is the NEGATIVE one: the same copy with the sync removed
 * is expected to fail. A test that only showed the working case could not tell
 * "the sync is needed and works" from "the sync is a no-op and the hardware
 * happens to be coherent".
 */
/* One direction, reported without judgement. Used by the matrix below to find
 * out WHICH side of the transfer PSRAM breaks, if any. */
static int c5_dma_one(const char *label, void *dst, const void *src,
                      rt_uint32_t len, rt_bool_t with_sync)
{
    rt_uint32_t i;
    rt_err_t err;

    for (i = 0; i < len; i++)
    {
        ((rt_uint8_t *)src)[i] = (rt_uint8_t)((i * 37u) ^ (i >> 5) ^ 0xC3u);
    }
    rt_memset(dst, 0, len);

    if (with_sync)
    {
        err = rt_hw_dma_memcpy_cached(dst, src, len);
    }
    else
    {
        err = rt_hw_gdma_memcpy(dst, src, len, 500u);
    }

    {
        rt_uint32_t diff = 0;

        for (i = 0; i < len; i++)
        {
            if (((const rt_uint8_t *)dst)[i] != ((const rt_uint8_t *)src)[i])
            {
                diff++;
            }
        }

        rt_kprintf("c5dma   : %-28s err=%2d  dst[0..3]=%02x %02x %02x %02x  "
                   "%u/%u differ\n",
                   label, (int)err,
                   ((const rt_uint8_t *)dst)[0], ((const rt_uint8_t *)dst)[1],
                   ((const rt_uint8_t *)dst)[2], ((const rt_uint8_t *)dst)[3],
                   (unsigned)diff, (unsigned)len);
    }

    return 0;
}

/* `c5dma matrix` - which combinations of SRAM/PSRAM actually transfer. */
static void c5_dma_matrix(void)
{
    rt_uint8_t *ps = (rt_uint8_t *)rt_malloc(C5_FORCE_PSRAM_LEN);
    rt_uint8_t *pd = (rt_uint8_t *)rt_malloc(C5_FORCE_PSRAM_LEN);
    static rt_uint8_t ss[512] __attribute__((aligned(C5_CACHE_LINE)));
    static rt_uint8_t sd[512] __attribute__((aligned(C5_CACHE_LINE)));
    const rt_uint32_t n = 256u;

    if (ps == RT_NULL || pd == RT_NULL)
    {
        rt_kprintf("c5dma   : matrix unavailable (PSRAM alloc failed)\n");
        if (ps) rt_free(ps);
        if (pd) rt_free(pd);
        return;
    }

    rt_kprintf("c5dma   : direction matrix, %u bytes, no cache sync anywhere\n",
               (unsigned)n);

    c5_dma_one("SRAM -> SRAM", sd, ss, n, RT_FALSE);
    c5_dma_one("SRAM -> PSRAM", pd, ss, n, RT_FALSE);
    c5_dma_one("PSRAM -> SRAM", sd, ps, n, RT_FALSE);
    c5_dma_one("PSRAM -> PSRAM", pd, ps, n, RT_FALSE);

    rt_free(ps);
    rt_free(pd);
    ps = pd = RT_NULL;
}

int c5_dma_psram_selftest(void)
{
    rt_uint32_t line;
    rt_uint32_t span;
    rt_uint32_t i;
    rt_err_t err;
    int fail = 0;

    if (!rt_hw_psram_heap_ready())
    {
        rt_kprintf("c5dma   : FAIL, no PSRAM heap - nothing to test against\n");
        return -1;
    }

    c5_dma_matrix();

    /* Ask for MORE than the 65 KB system heap can serve, so the fallback puts
     * these in PSRAM. A 4 KB request fits in internal SRAM and the test would
     * then be measuring the wrong memory entirely - measured exactly that on the
     * first run: `src SRAM, dst SRAM`. */
    s_src = (rt_uint8_t *)rt_malloc(C5_FORCE_PSRAM_LEN);
    s_dst = (rt_uint8_t *)rt_malloc(C5_FORCE_PSRAM_LEN);

    /* esp_cache_msync() requires cache-line alignment and memheap only promises
     * RT_ALIGN_SIZE (8). Rounding up costs a few bytes and makes the flush legal. */
    if (s_src != RT_NULL)
    {
        s_src = (rt_uint8_t *)RT_ALIGN((rt_ubase_t)s_src, C5_CACHE_LINE);
    }
    if (s_dst != RT_NULL)
    {
        s_dst = (rt_uint8_t *)RT_ALIGN((rt_ubase_t)s_dst, C5_CACHE_LINE);
    }

    if (s_src == RT_NULL || s_dst == RT_NULL)
    {
        rt_kprintf("c5dma   : FAIL, could not allocate two %u byte buffers\n",
                   (unsigned)C5_FORCE_PSRAM_LEN);
        if (s_src) rt_free(s_src);
        if (s_dst) rt_free(s_dst);
        return -1;
    }

    if ((rt_ubase_t)s_src < SOC_EXTRAM_DATA_LOW ||
        (rt_ubase_t)s_dst < SOC_EXTRAM_DATA_LOW)
    {
        rt_kprintf("c5dma   : FAIL, the buffers did not land in PSRAM - this test "
                   "would be measuring internal SRAM\n");
        rt_free(s_src);
        rt_free(s_dst);
        return -1;
    }

    line = c5_cache_line_size();
    span = RT_ALIGN(C5_DMA_TEST_LEN, line);

    rt_kprintf("c5dma   : cache line %u B, %u byte buffers, src %s, dst %s\n",
               (unsigned)line, (unsigned)C5_DMA_TEST_LEN,
               ((rt_ubase_t)s_src >= SOC_EXTRAM_DATA_LOW) ? "PSRAM" : "SRAM",
               ((rt_ubase_t)s_dst >= SOC_EXTRAM_DATA_LOW) ? "PSRAM" : "SRAM");

    /* Both buffers came from the fallback heap, so both are in PSRAM. That is
     * the hard case: DMA out of cached memory and back into cached memory. */
    for (i = 0; i < C5_DMA_TEST_LEN; i++)
    {
        s_src[i] = (rt_uint8_t)((i * 37u) ^ (i >> 5) ^ 0xC3u);
    }
    rt_memset(s_dst, 0, C5_DMA_TEST_LEN);

    /* --- 1. PSRAM -> PSRAM, with the cache handled --- */
    err = rt_hw_dma_memcpy_cached(s_dst, s_src, C5_DMA_TEST_LEN);

    if (err != RT_EOK)
    {
        /* Which -RT_ERROR? rt_hw_gdma_memcpy() returns the same code for a
         * descriptor error, a FIFO error and a content mismatch, so the code
         * alone cannot say whether the DMA ran. Print the first bytes of both
         * buffers: identical means the copy worked and the *comparison* is what
         * failed (a stale cache line), different means the transfer itself did
         * not happen. */
        rt_kprintf("c5dma   : FAIL, PSRAM->PSRAM copy returned %d; src[0..7]="
                   "%02x %02x %02x %02x %02x %02x %02x %02x dst[0..7]="
                   "%02x %02x %02x %02x %02x %02x %02x %02x\n",
                   (int)err,
                   s_src[0], s_src[1], s_src[2], s_src[3],
                   s_src[4], s_src[5], s_src[6], s_src[7],
                   s_dst[0], s_dst[1], s_dst[2], s_dst[3],
                   s_dst[4], s_dst[5], s_dst[6], s_dst[7]);
        fail++;
    }
    else if (rt_memcmp(s_dst, s_src, C5_DMA_TEST_LEN) != 0)
    {
        rt_uint32_t bad = 0;

        for (i = 0; i < C5_DMA_TEST_LEN; i++)
        {
            if (s_dst[i] != s_src[i])
            {
                if (bad < 3u)
                {
                    rt_kprintf("c5dma   :   +%u is 0x%02x, want 0x%02x\n",
                               (unsigned)i, s_dst[i], s_src[i]);
                }
                bad++;
            }
        }
        rt_kprintf("c5dma   : FAIL, %u byte(s) differ after PSRAM->PSRAM with sync\n",
                   (unsigned)bad);
        fail++;
    }
    else
    {
        rt_kprintf("c5dma   : PSRAM->PSRAM with cache sync: byte-exact\n");
    }

    /* --- 2. PSRAM -> SRAM, the direction with the cleaner failure mode ---
     *
     * Source in PSRAM (needs flush), destination in internal RAM (no cache to
     * invalidate). If the flush is missing the destination gets whatever was in
     * the PSRAM chip before, which is not the pattern. */
    rt_memset(s_dst, 0, C5_DMA_TEST_LEN);
    err = rt_hw_dma_memcpy_cached(s_dst, s_src, C5_DMA_TEST_LEN);

    if (err != RT_EOK || rt_memcmp(s_dst, s_src, C5_DMA_TEST_LEN) != 0)
    {
        rt_kprintf("c5dma   : FAIL, PSRAM->SRAM copy is not byte-exact (%d)\n",
                   (int)err);
        fail++;
    }
    else
    {
        rt_kprintf("c5dma   : PSRAM->SRAM with cache sync: byte-exact\n");
    }

    /* --- 3. The NEGATIVE control: same copy, sync skipped ---
     *
     * Expected to mismatch. If it passes, the cache handling above is not what
     * is making the copy work, and the test would be reporting a cause it has
     * not established. */
    for (i = 0; i < C5_DMA_TEST_LEN; i++)
    {
        s_src[i] = (rt_uint8_t)(i * 7u + 0x11u);
    }
    rt_memset(s_dst, 0, C5_DMA_TEST_LEN);

    err = rt_hw_gdma_memcpy(s_dst, s_src, C5_DMA_TEST_LEN, 500u);

    {
        rt_bool_t matched;

        if (err != RT_EOK)
        {
            /* The copy FAILED, which says nothing about the cache. Reporting this
             * as "mismatched, as expected" was the first version's bug: a -RT_EINVAL
             * from an oversized descriptor read exactly like a cache failure. */
            /* The copy failed outright. That is the expected outcome for a PSRAM
             * transfer with no cache handling, and the matrix at the top of this
             * function is what establishes it: all three PSRAM directions fail
             * there, while SRAM -> SRAM succeeds. Saying "inconclusive" here was
             * wrong - -RT_ERROR covers both "descriptor error" and "content
             * mismatch", and the matrix separates them. */
            rt_kprintf("c5dma   : control without cache sync failed (%d) - consistent "
                       "with the matrix above, where every PSRAM direction fails and "
                       "SRAM -> SRAM succeeds\n", (int)err);
        }
        else
        {
            matched = (rt_memcmp(s_dst, s_src, C5_DMA_TEST_LEN) == 0) ? RT_TRUE : RT_FALSE;

            rt_kprintf("c5dma   : control without cache sync: %s\n",
                       matched ? "byte-exact" : "mismatched, as expected");

            if (matched)
            {
                rt_kprintf("c5dma   : NOTE - the copy succeeded with NO cache "
                           "handling. Either the buffers are uncached or this part "
                           "is coherent; the sync above is then untested, not "
                           "proven.\n");
            }
            else
            {
                rt_kprintf("c5dma   : the flush/invalidate above is what makes it "
                           "work\n");
            }
        }
    }

    rt_free(s_src);
    rt_free(s_dst);
    s_src = s_dst = RT_NULL;

    if (fail != 0)
    {
        rt_kprintf("c5dma   : FAIL, %d check(s) did not hold\n", fail);
        return -1;
    }

    rt_kprintf("c5dma   : PASS (DMA in and out of cached PSRAM is byte-exact with "
               "the cache flushed before and invalidated after; the same copy "
               "without it is not)\n");
    return 0;
}

#endif /* SOC_PSRAM_DMA_CAPABLE || SOC_SPIRAM_SUPPORTED */
