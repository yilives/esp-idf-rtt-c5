/*
 * PSRAM proof, and the board's RGB LED.
 *
 * Two things that are easy to get wrong on this board and hard to notice:
 *
 * 1. PSRAM. The module carries 16 MB of Octal PSRAM, but "CONFIG_SPIRAM=y and it
 *    booted" proves almost nothing: the region has to be mapped, reachable
 *    through heap_caps_malloc(MALLOC_CAP_SPIRAM), and actually retaining data.
 *    `s31psram` allocates a block, writes a position-dependent pattern, reads it
 *    back through a volatile pointer (so the compiler cannot fold the check
 *    away), and reports the heap numbers from both pools.
 *
 * 2. The SK6805-2427 single-wire RGB LED on IO60. It is a WS2812-class part:
 *    each bit is a 1.25 us pulse, high 0.4 us for a 0 and 0.8 us for a 1, and
 *    the frame ends with a >50 us low. That timing is generated here by
 *    counting CPU cycles with interrupts masked, which costs 30 us of latency
 *    for a 24-bit frame - small enough to be acceptable and small enough to be
 *    auditable. RMT would be the "proper" way and is what IDF uses, but an RMT
 *    driver is a whole peripheral abstraction for one LED; the bit-bang version
 *    is 20 lines and cannot silently desynchronise, because a wrong cycle count
 *    shows up as the wrong colour instead of as nothing.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "soc/gpio_struct.h"
#include "hal/gpio_ll.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_clk.h"

#include "port_board.h"

/* ---------------------------------------------------------------- PSRAM -- */

static void st_psram_report(rt_bool_t ok, rt_uint32_t size, rt_uint32_t errors)
{
    if (ok)
    {
        rt_kprintf("s31psram: PASS, %u KB written and read back\n",
                   (unsigned)(size / 1024u));
    }
    else
    {
        rt_kprintf("s31psram: FAIL, %u bad bytes in %u KB\n",
                   (unsigned)errors, (unsigned)(size / 1024u));
    }
}

static int cmd_psram(int argc, char **argv)
{
    enum { LEN = 64 * 1024 };
    rt_uint32_t size = LEN;
    rt_uint8_t *blk;
    rt_uint32_t i;
    rt_uint32_t errors = 0;

    if (argc >= 2)
    {
        rt_int32_t k = atoi(argv[1]);

        if (k > 0)
        {
            size = (rt_uint32_t)k * 1024u;
        }
    }

    rt_kprintf("s31psram: internal heap free=%u, psram free=%u\n",
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) == 0)
    {
        rt_kprintf("s31psram: FAIL, no PSRAM in the heap at all\n");
        rt_kprintf("  check CONFIG_SPIRAM, and the boot log for\n");
        rt_kprintf("  'oct_psram: ... density' and 'MSPI Timing'\n");
        return -1;
    }

    blk = (rt_uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (blk == RT_NULL)
    {
        rt_kprintf("s31psram: FAIL, cannot allocate %u bytes in PSRAM\n",
                   (unsigned)size);
        return -1;
    }

    rt_kprintf("s31psram: allocated %u bytes at %p (psram)\n",
               (unsigned)size, blk);

    /* Position-dependent pattern: a plain memset would pass even if the block
     * were aliased onto itself, a pattern cannot. */
    for (i = 0; i < size; i++)
    {
        blk[i] = (rt_uint8_t)((i * 7u) ^ (i >> 8));
    }

    /* Volatile read-back so the compiler cannot eliminate the store/load pair. */
    for (i = 0; i < size; i++)
    {
        rt_uint8_t want = (rt_uint8_t)((i * 7u) ^ (i >> 8));

        if (*(volatile rt_uint8_t *)&blk[i] != want)
        {
            if (errors == 0)
            {
                rt_kprintf("s31psram: first mismatch at +%u: got 0x%02x want 0x%02x\n",
                           (unsigned)i, blk[i], want);
            }
            errors++;
        }
    }

    heap_caps_free(blk);

    st_psram_report(errors == 0, size, errors);
    return errors == 0 ? 0 : -1;
}
MSH_CMD_EXPORT_ALIAS(cmd_psram, s31psram, verify the 16 MB Octal PSRAM);

/* ---------------------------------------------------------------- init -- */

/*
 * The linker needs something to hold this object.
 *
 * Both commands are registered through MSH_CMD_EXPORT_ALIAS, i.e. by data in
 * the KEEP()'d FSymTab section, and nothing references any *symbol* in this
 * file. That is the same --gc-sections trap that made `list` disappear from the
 * shell earlier (log section 26.1): the member sits in librtthread.a, the
 * section survives, but the object is never pulled out of the archive unless a
 * global symbol is referenced. `board_extra.c.obj` was compiled and still absent
 * from the map, which is what gave it away.
 */
void s31_board_extra_link_anchor(void)
{
    /* Nothing to set up: the LED pin is configured on use (so it does not fight
     * the SPI LCD if IO60 is ever repurposed) and PSRAM is already up by the
     * time the scheduler runs. This exists purely as the owner's handle. */
}


/*
 * Cycle counting. The core runs at CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ (320), and
 * mcycle counts at that rate, so one microsecond is 320 cycles. Deriving it
 * from the config rather than hard-coding keeps the timing correct if the clock
 * changes - a wrong divisor produces recognisably wrong colours, not silence.
 */
#define LED_MCYCLE_PER_US       (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ)

static inline rt_uint32_t led_cycles(void)
{
    rt_uint32_t v;

    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));

    return v;
}

/* One WS2812 bit: 0 -> 0.4 us high / 0.85 us low, 1 -> 0.8 us high / 0.45 us
 * low. The margins are deliberately shifted away from the ±150 ns the parts
 * tolerate, because the loop overhead below is not free. */
static void led_send_bit(rt_uint32_t one)
{
    rt_uint32_t t0 = led_cycles();
    rt_uint32_t high = one ? (LED_MCYCLE_PER_US * 8u / 10u)
                           : (LED_MCYCLE_PER_US * 4u / 10u);
    rt_uint32_t total = LED_MCYCLE_PER_US * 5u / 4u;   /* 1.25 us */

    gpio_ll_set_level(&GPIO, BOARD_RGB_LED_PIN, 1);
    while ((rt_uint32_t)(led_cycles() - t0) < high)
    {
    }
    gpio_ll_set_level(&GPIO, BOARD_RGB_LED_PIN, 0);
    while ((rt_uint32_t)(led_cycles() - t0) < total)
    {
    }
}

static void led_send_byte(rt_uint8_t b)
{
    int i;

    for (i = 7; i >= 0; i--)
    {
        led_send_bit((b >> i) & 1u);
    }
}

/* SK6805/WS2812 order on this board is G,R,B (the schematic names the part
 * SK6805-2427; both that and WS2812 take GRB). */
static int cmd_led(int argc, char **argv)
{
    rt_uint32_t r = 0, g = 0, b = 0;
    rt_base_t level;

    if (argc >= 4)
    {
        r = (rt_uint32_t)atoi(argv[1]);
        g = (rt_uint32_t)atoi(argv[2]);
        b = (rt_uint32_t)atoi(argv[3]);
    }
    else if (argc >= 2 && rt_strcmp(argv[1], "off") == 0)
    {
        /* fall through with 0,0,0 */
    }
    else
    {
        rt_kprintf("usage: s31led <r> <g> <b>   (0..255 each), or s31led off\n");
        rt_kprintf("       s31led 0 0 0    off\n");
        rt_kprintf("       s31led 32 0 0   dim red\n");
        return -1;
    }

    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;

    gpio_ll_func_sel(&GPIO, (uint8_t)BOARD_RGB_LED_PIN, PIN_FUNC_GPIO);
    gpio_ll_output_enable(&GPIO, (uint8_t)BOARD_RGB_LED_PIN);
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)BOARD_RGB_LED_PIN, false, false);

    /* The whole frame has to be sent with interrupts masked: a tick in the
     * middle of a bit would stretch it past the 1.25 us period and the rest of
     * the frame would be sampled as nonsense. 24 bits x 1.25 us = 30 us. */
    level = rt_hw_interrupt_disable();

    led_send_byte((rt_uint8_t)g);
    led_send_byte((rt_uint8_t)r);
    led_send_byte((rt_uint8_t)b);

    /* Latch: the part needs >50 us of low to accept the frame. */
    rt_hw_interrupt_enable(level);
    rt_thread_mdelay(1);

    rt_kprintf("s31led: gpio %d <- R=%u G=%u B=%u\n",
               BOARD_RGB_LED_PIN, (unsigned)r, (unsigned)g, (unsigned)b);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_led, s31led, set the on-board SK6805 RGB LED);
