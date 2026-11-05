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

    if (test_kb > 1024u)
    {
        test_kb = 1024u;
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

#endif /* SOC_SPIRAM_SUPPORTED */


