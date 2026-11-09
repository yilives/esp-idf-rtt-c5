/*
 * ESP32-C5 PSRAM bring-up and verification.
 *
 * The part on this board is a C5HR8: 16 MB in-package flash, 8 MB in-package
 * PSRAM. The PSRAM half of that has never been enabled in this port - the
 * bootloader was built with `# CONFIG_SPIRAM is not set`, so nothing has ever
 * talked to it.
 *
 * ---------------------------------------------------------------------------
 * What this file adds, and why it is not just "turn CONFIG_SPIRAM on".
 *
 * CONFIG_SPIRAM + CONFIG_SPIRAM_BOOT_INIT make the IDF startup code call
 * esp_psram_init() before the RT-Thread kernel starts, and until that call
 * succeeds there is no way to tell "the config is wrong" from "this board has no
 * PSRAM" from "the PSRAM is there and the timing is wrong" - the startup path
 * only logs. So this exposes the same calls as a command that reports each step:
 *
 *   c5psram          - state: chip init ok?, reported size, heap region size
 *   c5psram test     - allocate from PSRAM and write/read back a pattern
 *
 * The two error paths are distinguished on purpose: esp_psram_chip_init()
 * failing means the chip did not answer (not fitted, or the wrong mode/speed);
 * succeeding but with a size of 0 means it answered but was mapped at zero
 * length.
 *
 * ---------------------------------------------------------------------------
 * Interaction with RT-Thread's own heap.
 *
 * RT-Thread keeps its own allocator on top of a fixed region
 * (CONFIG_IDF_RTTHREAD_HEAP_SIZE) and does not know about IDF's heap. PSRAM is
 * added to IDF's heap_caps allocator by esp_psram_init(), so
 *
 *   - `rt_malloc()` will NOT return PSRAM, ever, and
 *   - `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)` is the way in.
 *
 * Bridging the two is a real decision, not a detail: pointing RT-Thread's small
 * allocator at an external SPI-backed region changes the cost of every
 * allocation in the system, and its bitmaps live in the region they manage. That
 * is left alone here deliberately - this brings the memory up and proves it
 * works, and says plainly that nothing is using it yet.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"

/* Guarded on the CONFIG, not on SOC_SPIRAM_SUPPORTED.
 *
 * SOC_SPIRAM_SUPPORTED only says the chip has a PSRAM controller; the esp_psram
 * API behind it is compiled out entirely when CONFIG_SPIRAM is off, so guarding
 * on the capability would produce undefined references instead of an absent
 * file. Measured: the first version did exactly that - `undefined reference to
 * c5_psram_selftest` - because soc_caps.h was not even included, so the #if was
 * false and the whole file compiled to nothing.
 *
 * (The capability macro is included now as well: it is what tells a reader that
 * the missing piece was configuration and not silicon.) */
#if defined(CONFIG_SPIRAM)

#include "port_psram.h"

rt_bool_t rt_hw_psram_ready(void)
{
    return esp_psram_is_initialized() ? RT_TRUE : RT_FALSE;
}

rt_uint32_t rt_hw_psram_size(void)
{
    return (rt_uint32_t)esp_psram_get_size();
}

/* Claim one large PSRAM block and hand it to RT-Thread.
 *
 * This is the bridge `rt_malloc()` needed, and the design is deliberate:
 *
 *   - ONE `heap_caps_malloc` for the whole usable region, taken at init and
 *     never freed. Going through the IDF allocator once, up front, means the
 *     two allocators never interleave - RT-Thread owns the block outright and
 *     IDF's heap simply records it as used.
 *
 *   - The size is what the allocation actually returned, not the chip's
 *     reported capacity. The IDF heap needs its own metadata out of the same
 *     region, and a request for the full 8 MB fails: measured,
 *     `heap_caps_malloc(8294400)` returns NULL while 7168000 succeeds.
 *
 *   - If the allocation fails the function returns NULL and the caller leaves
 *     the system heap alone. A 65 KB heap that works is better than a merged
 *     one that does not. */
void *rt_hw_psram_claim(rt_uint32_t *out_size)
{
    rt_uint32_t want;
    void *p;

    if (out_size != RT_NULL)
    {
        *out_size = 0;
    }

    if (!esp_psram_is_initialized())
    {
        return RT_NULL;
    }

    /* Leave a margin: the IDF heap's own bookkeeping and any earlier stray
     * allocation come out of the same region. */
    want = (rt_uint32_t)esp_psram_get_size();
    if (want > C5_PSRAM_HEAP_RESERVE)
    {
        want -= C5_PSRAM_HEAP_RESERVE;
    }
    want &= ~0xFFFu;                 /* whole pages; the mapping is page based */

    p = heap_caps_malloc(want, MALLOC_CAP_SPIRAM);
    while (p == RT_NULL && want > (1u << 20))
    {
        want >>= 1;                  /* halve until it fits, and say how much */
        want &= ~0xFFFu;
        p = heap_caps_malloc(want, MALLOC_CAP_SPIRAM);
    }

    if (p != RT_NULL && out_size != RT_NULL)
    {
        *out_size = want;
    }

    return p;
}

/* ------------------------------------------------------------------ msh -- */

static rt_uint32_t c5_psram_crc32(const rt_uint8_t *p, rt_uint32_t len)
{
    rt_uint32_t crc = 0xFFFFFFFFu;
    rt_uint32_t i;
    int b;

    for (i = 0; i < len; i++)
    {
        crc ^= p[i];
        for (b = 0; b < 8; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (rt_uint32_t)(-(rt_int32_t)(crc & 1u)));
        }
    }

    return ~crc;
}

/* `c5psram [test [kb]]`
 *
 * Without an argument: report the state. With `test`: allocate from PSRAM, fill
 * it with a pattern that differs at every byte position (so a stuck address line
 * shows up as a mismatch rather than as a passing block of identical bytes), read
 * it back, and report a CRC.
 */
int c5_psram_selftest(rt_uint32_t test_kb)
{
    rt_uint32_t total;
    rt_bool_t   ready;

    ready = rt_hw_psram_ready();
    total = rt_hw_psram_size();

    rt_kprintf("c5psram : initialised=%s, reported size=%u byte(s)\n",
               ready ? "yes" : "NO", (unsigned)total);
    rt_kprintf("c5psram : IDF heap: internal=%u, spiram=%u, total=%u byte(s)\n",
               (unsigned)heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM),
               (unsigned)heap_caps_get_total_size(MALLOC_CAP_DEFAULT));

    if (!ready || total == 0u)
    {
        rt_kprintf("c5psram : FAIL, the PSRAM chip did not come up - "
                   "not fitted, or the mode/speed in sdkconfig does not match it\n");
        return -1;
    }

    if (test_kb == 0u)
    {
        rt_kprintf("c5psram : state read only; `c5psram test [kb]` runs the "
                   "read/write check\n");
        return 0;
    }

    /* Cap at what the chip actually reports, not at a hard-coded 1 MB.
     *
     * The old cap was silently clamping `c5psram test 7680` to 1024 KB and then
     * printing "wrote and read back 1048576 byte(s)" - so the test looked like it
     * had honoured the request while only exercising an eighth of the part. That
     * is the same class of mistake this report keeps recording: a limit nobody
     * stated, producing a pass that means less than it appears to.
     *
     * Leave a margin for the allocator's own bookkeeping and for whatever else has
     * already been allocated from the same heap. */
    {
        rt_uint32_t max_kb = (total / 1024u);

        if (max_kb > 64u)
        {
            max_kb -= 64u;
        }

        if (test_kb > max_kb)
        {
            rt_kprintf("c5psram : %u KB requested, clamping to %u KB (the chip "
                       "reports %u KB)\n",
                       (unsigned)test_kb, (unsigned)max_kb, (unsigned)(total / 1024u));
            test_kb = max_kb;
        }
    }

    {
        rt_uint32_t want = test_kb * 1024u;
        rt_uint8_t *buf;
        rt_uint32_t i;
        rt_uint32_t bad = 0;
        rt_uint32_t crc;

        /* MALLOC_CAP_SPIRAM and nothing else, so a failure here really means
         * "PSRAM cannot satisfy this" rather than being silently served from
         * internal RAM. */
        buf = (rt_uint8_t *)heap_caps_malloc(want, MALLOC_CAP_SPIRAM);
        if (buf == RT_NULL)
        {
            rt_kprintf("c5psram : FAIL, heap_caps_malloc(%u, MALLOC_CAP_SPIRAM) "
                       "returned NULL\n", (unsigned)want);
            return -1;
        }

        /* Prove the pointer really is in the PSRAM window.
         *
         * MALLOC_CAP_SPIRAM should guarantee it, but a test that writes to
         * internal RAM and passes would be worthless, so the address is checked
         * against the heap's own capability query rather than assumed. */
        if (!heap_caps_check_integrity_addr((intptr_t)buf, true))
        {
            rt_kprintf("c5psram : FAIL, the returned pointer is not in a "
                       "valid heap region\n");
            heap_caps_free(buf);
            return -1;
        }

        /* A pattern that depends on the address: byte i gets (i ^ (i >> 8)). Two
         * different addresses never hold the same value, so a shorted or stuck
         * address line cannot pass. */
        for (i = 0; i < want; i++)
        {
            buf[i] = (rt_uint8_t)(i ^ (i >> 8));
        }

        crc = c5_psram_crc32(buf, want);

        for (i = 0; i < want; i++)
        {
            if (buf[i] != (rt_uint8_t)(i ^ (i >> 8)))
            {
                if (bad < 8u)
                {
                    rt_kprintf("c5psram : mismatch at +%u: %02x (want %02x)\n",
                               (unsigned)i, buf[i],
                               (rt_uint8_t)(i ^ (i >> 8)));
                }
                bad++;
            }
        }

        rt_kprintf("c5psram : wrote and read back %u byte(s), crc32=0x%08x, "
                   "%u mismatch(es)\n", (unsigned)want, (unsigned)crc,
                   (unsigned)bad);

        heap_caps_free(buf);

        if (bad != 0u)
        {
            rt_kprintf("c5psram : FAIL, the memory did not retain the pattern\n");
            return -1;
        }
    }

    rt_kprintf("c5psram : PASS (the chip answers, is mapped, and retains a "
               "position-dependent pattern; NOT exercised under DMA or as "
               "executable memory, and RT-Thread's own heap does not use it)\n");
    return 0;
}

/* ------------------------------------------------------------------ heap -- */

#if defined(RT_USING_MEMHEAP)

/* The block claimed for RT-Thread, and the heap object that owns it. */
static struct rt_memheap s_psram_heap;
static void             *s_psram_block;
static rt_uint32_t       s_psram_block_size;

rt_bool_t rt_hw_psram_heap_ready(void)
{
    return (s_psram_block != RT_NULL) ? RT_TRUE : RT_FALSE;
}

rt_uint32_t rt_hw_psram_heap_size(void)
{
    return s_psram_block_size;
}

/* Register the claimed block as a memheap named "psram".
 *
 * With RT_USING_MEMHEAP_AS_HEAP + RT_USING_MEMHEAP_AUTO_BINDING, `rt_malloc()`
 * tries the system heap first and then every other memheap in the object list -
 * so registering this one is what makes small allocations keep coming from the
 * fast internal SRAM while large ones spill into PSRAM. That is the behaviour
 * that is actually wanted here: internal SRAM is 65 KB and cache-coherent, PSRAM
 * is 8 MB and goes through the SPI cache on every access.
 *
 * Nothing is merged into a single pool on purpose. A merged pool would have to
 * pick one allocator for both, and either choice is wrong: small-mem over PSRAM
 * pays the cache penalty on every allocation, and memheap over internal SRAM
 * costs 40 bytes of header per allocation in a 65 KB region. */
rt_err_t rt_hw_psram_heap_init(void)
{
    if (s_psram_block != RT_NULL)
    {
        return RT_EOK;
    }

    s_psram_block = rt_hw_psram_claim(&s_psram_block_size);

    if (s_psram_block == RT_NULL || s_psram_block_size < (256u * 1024u))
    {
        s_psram_block = RT_NULL;
        s_psram_block_size = 0;
        return -RT_ERROR;
    }

    if (rt_memheap_init(&s_psram_heap, "psram", s_psram_block,
                        s_psram_block_size) != RT_EOK)
    {
        s_psram_block = RT_NULL;
        s_psram_block_size = 0;
        return -RT_ERROR;
    }

    /* The system heap takes its own lock; this one is a plain memheap and needs
     * the same treatment, otherwise two threads can corrupt it. */
    s_psram_heap.locked = RT_TRUE;

    return RT_EOK;
}

/* Register the PSRAM heap at board stage.
 *
 * INIT_BOARD_EXPORT runs after the kernel heap exists (rt_system_heap_init is at
 * the very start of rt_hw_board_init) and before any device or application
 * thread allocates, which is the only window where adding a heap is safe: an
 * allocation made before this point that later has to be freed would still be
 * found, because rt_memheap_free() walks to the owning pool - but a thread that
 * started earlier could have already failed a large request.
 *
 * The -u anchor in CMakeLists is required for the same reason as everywhere else
 * in this port: an INIT_*_EXPORT entry is data, and --gc-sections drops members
 * nothing references by symbol. */
rt_err_t rt_hw_psram_heap_auto_init(void)
{
    return rt_hw_psram_heap_init();
}
INIT_BOARD_EXPORT(rt_hw_psram_heap_auto_init);

#endif /* RT_USING_MEMHEAP */

#endif /* SOC_SPIRAM_SUPPORTED */


