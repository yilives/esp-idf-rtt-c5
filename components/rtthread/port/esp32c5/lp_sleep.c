/*
 * ESP32-C5 low-power behaviour: what actually survives a deep sleep.
 *
 * The board has two cores and the port has so far exercised one of them. The
 * question this file answers is not "does the LP core run" - round 16 proved that
 * with a shared-variable counter - but "what does the low-power mode actually do
 * to the things this port already relies on".
 *
 * That is a more useful question here, because a deep sleep on this part:
 *
 *   - powers down the HP core, so every RT-Thread thread, the heap and the tick
 *     are gone;
 *   - powers down the 8 MB PSRAM, which is why `rt_hw_psram_ready()` returning
 *     true at boot says nothing about it coming back after a wake;
 *   - keeps RTC slow memory alive, which is the only place a marker can survive.
 *
 * So the test writes a marker into RTC memory, records whether PSRAM held its
 * contents across the sleep, sleeps for a bounded time, and on wake compares.
 *
 * ---------------------------------------------------------------------------
 * Why the result is read as a reboot rather than a return.
 *
 * `esp_deep_sleep_start()` does not return. On wake the chip restarts from the
 * bootloader, so the report appears on the *next* boot, printed by init code that
 * checks the RTC marker. The msh command therefore only arms and sleeps; the
 * verdict shows up in the boot banner of the following reset. Anything else -
 * printing a result inside the same boot - would be a measurement of nothing.
 *
 * ---------------------------------------------------------------------------
 * What is NOT proven, stated up front:
 *
 *   - current consumption. Nothing on this board measures it, and a number
 *     without a measurement would be a guess.
 *   - that any particular wake source other than the timer works.
 *   - light sleep, which keeps the HP core alive and is a different mechanism.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_system.h"

#if defined(SOC_PM_SUPPORTED) || defined(SOC_LP_CORE_SUPPORTED)

#include "esp_heap_caps.h"

#include "port_lp.h"
#include "port_psram.h"

/* RTC_NOINIT_ATTR: not zeroed by the bootloader and not touched by a normal
 * reset, which is the only way a value can cross a deep sleep. `magic` guards
 * against reading garbage on a cold boot. */
#define C5_LP_SLEEP_MAGIC   0x4c50534c  /* "LPSL" */

RTC_NOINIT_ATTR static rt_uint32_t  s_sleep_magic;
RTC_NOINIT_ATTR static rt_uint32_t  s_sleep_count;
RTC_NOINIT_ATTR static rt_uint32_t  s_sleep_psram_crc;
RTC_NOINIT_ATTR static rt_uint32_t  s_sleep_psram_ok;
RTC_NOINIT_ATTR static rt_uint32_t  s_sleep_psram_addr;
RTC_NOINIT_ATTR static rt_uint32_t  s_sleep_psram_len;
RTC_NOINIT_ATTR static rt_uint32_t  s_sleep_wake_cause;

rt_bool_t rt_hw_lp_have_sleep_record(void)
{
    return (s_sleep_magic == C5_LP_SLEEP_MAGIC);
}

void rt_hw_lp_print_sleep_record(void)
{
    if (!rt_hw_lp_have_sleep_record())
    {
        return;
    }

    /* Read the wake cause HERE rather than at arm time.
     *
     * `esp_sleep_get_wakeup_cause()` is only meaningful after a wake, so a value
     * captured before `esp_deep_sleep_start()` is whatever that function returned
     * on the way down - measured: it printed 4069716927, which is not a wake
     * cause at all but an uninitialised RTC word read back. */
    s_sleep_wake_cause = (rt_uint32_t)esp_sleep_get_wakeup_cause();

    rt_kprintf("lp      : woke from deep sleep, cause=%u (%s), %u sleep(s) so far\n",
               (unsigned)s_sleep_wake_cause,
               (s_sleep_wake_cause == 4u) ? "timer" : "not a timer wake",
               (unsigned)s_sleep_count);

    /* Actually compare the bytes, and only claim a result when there is one to
     * claim. "NO" from a value that was never measured is a worse answer than
     * "not tested". */
    if (s_sleep_psram_addr == 0u || s_sleep_psram_len == 0u)
    {
        rt_kprintf("lp      : PSRAM retention NOT tested (no block was recorded "
                   "before the sleep)\n");
        return;
    }

    {
        const rt_uint8_t *p = (const rt_uint8_t *)(rt_ubase_t)s_sleep_psram_addr;
        rt_uint32_t i;
        rt_uint32_t crc = 0xFFFFFFFFu;
        rt_uint32_t bad = 0;

        for (i = 0; i < s_sleep_psram_len; i++)
        {
            rt_uint8_t want = (rt_uint8_t)(i * 7u + 0x31u);
            rt_uint32_t k;

            if (p[i] != want && bad < 4u)
            {
                rt_kprintf("lp      : PSRAM +%u is 0x%02x, wrote 0x%02x\n",
                           (unsigned)i, p[i], want);
                bad++;
            }

            crc ^= p[i];
            for (k = 0; k < 8u; k++)
            {
                crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
            }
        }

        crc ^= 0xFFFFFFFFu;

        if (crc == s_sleep_psram_crc)
        {
            rt_kprintf("lp      : PSRAM retained %u byte(s) across the sleep "
                       "(crc 0x%08x matches)\n",
                       (unsigned)s_sleep_psram_len, (unsigned)crc);
        }
        else
        {
            rt_kprintf("lp      : PSRAM did NOT retain its contents: crc 0x%08x, "
                       "wrote 0x%08x (%u shown)\n",
                       (unsigned)crc, (unsigned)s_sleep_psram_crc, (unsigned)bad);
        }
    }
}

/* Prepare, arm the timer and go down. Never returns. */
int c5_lp_sleep_selftest(rt_uint32_t seconds)
{
    if (seconds == 0u || seconds > 60u)
    {
        seconds = 3u;
    }

    rt_kprintf("lp      : deep sleep for %u s\n", (unsigned)seconds);

    /* Put a pattern in PSRAM and record its CRC before sleeping. If PSRAM loses
     * power the allocation is not merely freed - the block reads back as
     * different bytes - so the comparison on wake is a real test of retention
     * rather than of the allocator. */
    if (rt_hw_psram_ready())
    {
        rt_uint32_t len = 4096u;
        rt_uint8_t *buf = (rt_uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);

        if (buf != RT_NULL)
        {
            rt_uint32_t i;

            for (i = 0; i < len; i++)
            {
                buf[i] = (rt_uint8_t)(i * 7u + 0x31u);
            }

            {
                rt_uint32_t crc = 0xFFFFFFFFu;

                for (i = 0; i < len; i++)
                {
                    rt_uint32_t k;

                    crc ^= buf[i];
                    for (k = 0; k < 8u; k++)
                    {
                        crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
                    }
                }

                s_sleep_psram_crc = crc ^ 0xFFFFFFFFu;
            }

            /* The ADDRESS and the LENGTH are saved, and the block is not freed.
             *
             * An earlier version of this file deliberately discarded the address
             * on the reasoning that the heap is rebuilt after a sleep so a saved
             * pointer would be meaningless - and then printed "PSRAM held its
             * contents: NO" from a variable it never set. PSRAM on this part is a
             * fixed memory-mapped region, so the physical bytes at that address
             * are the same bytes after a wake; reading them directly bypasses the
             * allocator, which is exactly what "did the memory retain it" means. */
            s_sleep_psram_addr = (rt_uint32_t)(rt_ubase_t)buf;
            s_sleep_psram_len = len;
        }
        else
        {
            rt_kprintf("lp      : no PSRAM block available, retention not tested\n");
            s_sleep_psram_ok = 0u;
            s_sleep_psram_addr = 0u;
            s_sleep_psram_len = 0u;
        }
    }

    s_sleep_count = rt_hw_lp_have_sleep_record() ? (s_sleep_count + 1u) : 1u;
    s_sleep_magic = C5_LP_SLEEP_MAGIC;

    rt_kprintf("lp      : RTC marker set, entering deep sleep - the verdict appears "
               "on the NEXT boot\n");
    rt_kprintf("lp      : `esp_deep_sleep_start()` does not return; this console "
               "will drop\n");

    (void)esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ull);

    /* Nothing registered here needs a clean shutdown, and there is no scheduler
     * hook that survives: the HP core is powered down either way. */
    esp_deep_sleep_start();

    /* Not reached. */
    return -1;
}

#endif /* SOC_PM_SUPPORTED || SOC_LP_CORE_SUPPORTED */
