/*
 * General-purpose timers (TIMG) with a periodic callback.
 *
 * There is no general "hardware timer" device class to register into, but the reason
 * matters - an earlier version of this comment said "the code is gone", which is false.
 *
 * components/drivers/clock_time/ exists (1294 lines: clock_time_core.c, clock_timer.c,
 * clock_hrtimer.c, clock_boottime.c) behind RT_USING_CLOCK_TIME. It is not a general
 * timer class though: clock_timer.c implements ONE-SHOT/PERIOD *deadlines* against a
 * single system-wide clock. `ops->start(timer, cnt, mode)` counts `cnt` ticks of the
 * timer's own freq, the ISR decrements `cycles` and fires rt_clock_time_event_isr(),
 * and there is no per-timer callback or argument. It is also single-owner by design:
 * `static rt_clock_timer_t *_clock_timer_owner`, and only the first registered timer
 * with a `start` op becomes the clock source.
 *
 * This driver is a different thing: four independent channels, each with its own
 * callback and argument, started at a frequency in Hz and left running periodically.
 * Registering one of them as clock_time's owner would also redefine what
 * rt_clock_time_set_timeout() means for the whole system - a behavioural change with no
 * caller asking for it. So this stays a small direct API. The intended users are the
 * kind that need a deadline finer than the 1 ms tick: a bit-banged protocol, a sampling
 * loop, or jitter measurement.
 *
 * Layout: two TIMG groups, two timers each, so four independent channels.
 * Every timer has its own interrupt source (ETS_TIMERGRP0_T0/T1,
 * ETS_TIMERGRP1_T0/T1), so each channel gets its own handler allocation. The
 * handler still reads the group status register and dispatches on it, because
 * that register is per group: a spurious call for a timer that is not running
 * has to be harmless rather than clearing the other timer's pending bit.
 *
 * Timebase: XTAL (exactly CONFIG_XTAL_FREQ) divided down to 1 MHz, so one count
 * is one microsecond and the alarm value is simply 1000000 / frequency. APB
 * would be faster but its frequency moves with the clock tree, which would make
 * a requested frequency silently wrong after any DFS change.
 *
 * Note the contrast with s31_systimer_us() below: TIMG really is programmed to
 * 1 MHz by the prescaler in s31_timer_start(), while the SYSTIMER counter is a
 * 16 MHz raw counter that has to be converted. The two are not interchangeable.
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "soc/timer_group_struct.h"
#include "soc/interrupts.h"
#include "soc/systimer_struct.h"
#include "hal/timer_ll.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "esp_private/systimer.h"

#include "port_timer.h"

#define S31_TIMG_COUNT      2
#define S31_TIMG_TIMERS     2
#define S31_TIMG_BASE_HZ    1000000u    /* 1 MHz after the prescaler */

struct s31_timg_group
{
    timg_dev_t     *hw;
    intr_handle_t   intr[S31_TIMG_TIMERS];
    s31_timer_cb_t  cb[S31_TIMG_TIMERS];
    void           *arg[S31_TIMG_TIMERS];
};

static struct s31_timg_group s_group[S31_TIMG_COUNT];

static const int s_timg_irq_source[S31_TIMG_COUNT][S31_TIMG_TIMERS] =
{
    { ETS_TIMERGRP0_T0_INTR_SOURCE, ETS_TIMERGRP0_T1_INTR_SOURCE },
    { ETS_TIMERGRP1_T0_INTR_SOURCE, ETS_TIMERGRP1_T1_INTR_SOURCE },
};

rt_uint64_t s31_systimer_us(void)
{
    rt_uint32_t hi, lo, hi2;
    rt_uint64_t raw;

    /* Two things here were wrong, and the second one only shows up after the
     * first is fixed - so both are spelled out.
     *
     * 1. THE UNIT. The SYSTIMER counter does NOT tick at 1 MHz on this chip.
     *    S31 feeds it from XTAL (40 MHz) through a fixed 2.5 divider, so one raw
     *    count is 62.5 ns and the counter runs at 16 MHz. IDF's own target port
     *    says so and converts with `ticks / 16`
     *    (components/esp_hw_support/port/esp32s31/systimer.c). Reading the
     *    register and calling the result "microseconds" made every measurement
     *    16x too large - the 15999 us/tick that `s31test tick` reported and that
     *    tools/serial_time.py was written to chase.
     *
     * 2. WHICH COUNTER. This read `unit_val[0]`, the counter esp_timer owns
     *    (SYSTIMER_COUNTER_ESPTIMER). Nothing guarantees it is running at the
     *    point this is called - measured on the board: unit 0 reads 0 while unit
     *    1 counts - so a fixed unit 0 turns every interval into zero, which is
     *    worse than the 16x error because it looks like "no time passed".
     *    Unit 1 is SYSTIMER_COUNTER_OS_TICK: vSystimerSetup() enables it
     *    unconditionally before the first thread runs, and it is the same
     *    counter the tick alarm compares against, which is what makes this a
     *    meaningful reference for "how long did the tick take".
     *
     * systimer_ticks_to_us() rather than a literal 16: the ratio belongs to the
     * IDF target port, so a future divider change is followed rather than
     * missed. It is in the image anyway for esp_timer, so no new dependency. */
    do
    {
        hi  = SYSTIMER.unit_val[SYSTIMER_COUNTER_OS_TICK].hi.timer_unit_value_hi;
        lo  = SYSTIMER.unit_val[SYSTIMER_COUNTER_OS_TICK].lo.timer_unit_value_lo;
        hi2 = SYSTIMER.unit_val[SYSTIMER_COUNTER_OS_TICK].hi.timer_unit_value_hi;
    } while (hi != hi2);

    raw = ((rt_uint64_t)hi << 32) | lo;

    return (rt_uint64_t)systimer_ticks_to_us(raw);
}

static void IRAM_ATTR s31_timg_isr(void *arg)
{
    struct s31_timg_group *g = (struct s31_timg_group *)arg;
    rt_uint32_t status = timer_ll_get_intr_status(g->hw);

    /* Clear only the alarm bits this group actually latched. Writing back the
     * whole status register would also acknowledge bits belonging to the other
     * timer, whose handler then finds nothing pending. */
    rt_uint32_t pending = status & (TIMER_LL_EVENT_ALARM(0) | TIMER_LL_EVENT_ALARM(1));

    if (pending == 0)
    {
        return;
    }

    timer_ll_clear_intr_status(g->hw, pending);

    for (int t = 0; t < S31_TIMG_TIMERS; t++)
    {
        if (pending & TIMER_LL_EVENT_ALARM(t))
        {
            /* The hardware disables tx_alarm_en when the alarm fires, even with
             * auto-reload on, so the alarm has to be re-armed here or the
             * callback runs exactly once.
             *
             * Measured on the board: `s31timer 0 1000` reported
             * "got 1 Hz over 1000000 us, 1 ticks" - one interrupt, then silence,
             * for every requested rate. IDF's own gptimer isr does the same
             * re-arm (esp_driver_gptimer/src/gptimer.c:466-469, with the comment
             * "when alarm event happens, the alarm will be disabled
             * automatically by hardware"). Auto-reload only reloads the counter;
             * it does not re-enable the comparison. */
            timer_ll_enable_alarm(g->hw, t, true);

            if (g->cb[t] != RT_NULL)
            {
                g->cb[t](g->arg[t]);
            }
        }
    }
}

rt_err_t s31_timer_start(rt_uint32_t idx, rt_uint32_t freq_hz,
                         s31_timer_cb_t cb, void *arg)
{
    rt_uint32_t group = idx / S31_TIMG_TIMERS;
    rt_uint32_t timer = idx % S31_TIMG_TIMERS;
    struct s31_timg_group *g;
    timg_dev_t *hw;
    rt_uint32_t prescale;
    rt_uint64_t alarm;

    if (idx >= S31_TIMER_NUM || cb == RT_NULL || freq_hz == 0 ||
        freq_hz > S31_TIMG_BASE_HZ)
    {
        return -RT_EINVAL;
    }

    g  = &s_group[group];
    hw = g->hw;

    g->cb[timer]  = cb;
    g->arg[timer] = arg;

    _timer_ll_enable_clock(group, timer, true);
    _timer_ll_set_clock_source(group, timer, GPTIMER_CLK_SRC_XTAL);

    prescale = (CONFIG_XTAL_FREQ * 1000000u) / S31_TIMG_BASE_HZ;   /* 40 */
    timer_ll_set_clock_prescale(hw, timer, prescale);

    timer_ll_set_count_direction(hw, timer, GPTIMER_COUNT_UP);
    timer_ll_set_reload_value(hw, timer, 0);
    timer_ll_enable_auto_reload(hw, timer, true);

    /* The alarm value IS the period in 1 MHz counts - do not subtract one.
     *
     * Measured on the board: alarm = 199 produced 5024 Hz (1e6/199 = 5025), and
     * alarm = 4 produced 248975 Hz (1e6/4 = 250000). So the counter reloads to
     * tx_load = 0 on the cycle the comparison fires, and the distance between
     * two edges is exactly the alarm value. The old `alarm -= 1` therefore made
     * every request one count fast (0.1 % at 1 kHz, 25 % at 200 kHz).
     *
     * Round rather than truncate as well: plain division floors 1e6/600000 to 1,
     * which is why everything above 500 kHz used to come out as 1 MHz. */
    if (freq_hz > S31_TIMG_BASE_HZ)
    {
        rt_kprintf("s31timer: %u Hz is above the 1 MHz timebase\n",
                   (unsigned)freq_hz);
        return -RT_EINVAL;
    }

    alarm = ((rt_uint64_t)S31_TIMG_BASE_HZ + freq_hz / 2u) / freq_hz;
    if (alarm == 0)
    {
        alarm = 1;
    }

    timer_ll_set_alarm_value(hw, timer, alarm);
    timer_ll_enable_alarm(hw, timer, true);

    if (g->intr[timer] == RT_NULL)
    {
        /* Each timer owns its own source, so no sharing flags are needed.
         * Level 1 is enough: nothing here needs to preempt a higher-priority
         * source, and keeping it at the default level means the CLIC threshold
         * in board.c lets it through. */
        if (esp_intr_alloc(s_timg_irq_source[group][timer], ESP_INTR_FLAG_LEVEL1,
                           s31_timg_isr, g, &g->intr[timer]) != ESP_OK)
        {
            return -RT_ERROR;
        }
    }

    timer_ll_enable_intr(hw, TIMER_LL_EVENT_ALARM(timer), true);
    timer_ll_enable_counter(hw, timer, true);

    return RT_EOK;
}

/*
 * The esp_intr handle for a channel, so a caller that started a timer on another core can
 * enable the line on that core. esp_intr_alloc() installs and routes but does not enable;
 * RT-Thread's rt_hw_interrupt_umask() writes the CURRENT core's CLIC window, so it cannot
 * be used from elsewhere.
 */
/* The ESP interrupt source a channel was allocated on, or -1. Diagnostics only. */
int s31_timer_irq_source(rt_uint32_t idx)
{
    rt_uint32_t group = idx / S31_TIMG_TIMERS;
    rt_uint32_t timer = idx % S31_TIMG_TIMERS;

    if (idx >= S31_TIMER_NUM)
    {
        return -1;
    }

    return s_timg_irq_source[group][timer];
}

void *s31_timer_intr_handle(rt_uint32_t idx)
{
    rt_uint32_t group = idx / S31_TIMG_TIMERS;
    rt_uint32_t timer = idx % S31_TIMG_TIMERS;

    if (idx >= S31_TIMER_NUM)
    {
        return RT_NULL;
    }

    return (void *)s_group[group].intr[timer];
}

rt_err_t s31_timer_stop(rt_uint32_t idx)
{
    rt_uint32_t group = idx / S31_TIMG_TIMERS;
    rt_uint32_t timer = idx % S31_TIMG_TIMERS;
    struct s31_timg_group *g;

    if (idx >= S31_TIMER_NUM)
    {
        return -RT_EINVAL;
    }

    g = &s_group[group];

    timer_ll_enable_counter(g->hw, timer, false);
    timer_ll_enable_intr(g->hw, TIMER_LL_EVENT_ALARM(timer), false);
    timer_ll_enable_alarm(g->hw, timer, false);
    g->cb[timer]  = RT_NULL;
    g->arg[timer] = RT_NULL;

    return RT_EOK;
}

rt_uint64_t s31_timer_count(rt_uint32_t idx)
{
    if (idx >= S31_TIMER_NUM)
    {
        return 0;
    }

    return timer_ll_get_counter_value(s_group[idx / S31_TIMG_TIMERS].hw,
                                      idx % S31_TIMG_TIMERS);
}

static volatile rt_uint32_t s_probe_count;

static void s_probe_cb(void *arg)
{
    RT_UNUSED(arg);
    s_probe_count++;
}

static int cmd_timer(int argc, char **argv)
{
    rt_uint32_t idx = 0, hz = 1000;
    rt_uint64_t t0, t1;
    rt_uint32_t n;

    if (argc >= 2)
    {
        idx = (rt_uint32_t)atoi(argv[1]);
    }
    if (argc >= 3)
    {
        hz = (rt_uint32_t)atoi(argv[2]);
    }

    if (idx >= S31_TIMER_NUM)
    {
        rt_kprintf("usage: s31timer [0..%d] [hz]\n", S31_TIMER_NUM - 1);
        return -1;
    }

    s_probe_count = 0;
    if (s31_timer_start(idx, hz, s_probe_cb, RT_NULL) != RT_EOK)
    {
        rt_kprintf("s31timer: start failed (hz must be 1..1000000)\n");
        return -1;
    }

    /* Measure with SYSTIMER, which is a different peripheral from TIMG. Using
     * rt_tick_get() here would compare the timer against a value that itself
     * depends on a timer. */
    t0 = s31_systimer_us();
    rt_thread_mdelay(1000);
    t1 = s31_systimer_us();
    n  = s_probe_count;

    s31_timer_stop(idx);

    if (n == 0)
    {
        rt_kprintf("s31timer: FAIL, no interrupts from timer %d in 1 s\n", (int)idx);
        return -1;
    }

    {
        rt_uint64_t elapsed_us = t1 - t0;
        rt_uint32_t measured   = (rt_uint32_t)(((rt_uint64_t)n * 1000000u) / elapsed_us);

        /* Tolerance is the larger of 1 % and one whole count: at 2 Hz a one
         * second window can legitimately capture one edge or two, and a strict
         * percentage window reports a correct timer as broken. 1 % is still the
         * bar anywhere the window holds enough edges for it to mean anything. */
        {
            rt_int32_t slack = (rt_int32_t)(hz / 100);

            if (slack < 2)
            {
                slack = 2;
            }

            {
                int ok = (measured + (rt_uint32_t)slack >= hz) &&
                         (measured <= hz + (rt_uint32_t)slack);

                rt_kprintf("s31timer: timer %d want %d Hz, got %d Hz over %d us, "
                           "%d ticks -> %s\n",
                           (int)idx, (int)hz, (int)measured, (int)elapsed_us,
                           (int)n, ok ? "PASS" : "FAIL (outside tolerance)");
            }
        }
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_timer, s31timer, GPTimer periodic interrupt rate check);


/*
 * rt_hw_us_delay: the busy-wait rthw.h declares and kservice.c only stubs.
 *
 * The stub is `rt_weak` and logs "doesn't support for this board"; because it lives
 * in the same archive as this file, a `-u rt_hw_us_delay` cannot beat it - the weak
 * definition satisfies the undefined reference and nothing pulls the strong one in.
 * Measured: the symbol stayed `W` in the ELF after adding the flag. So the
 * definition lives here instead, in an object that is already linked for
 * rt_hw_timer_init.
 *
 * UNIT. The S31 SYSTIMER counts at 16 MHz (XTAL 40 MHz / 2.5), so a raw register
 * read is 62.5 ns. s31_systimer_us() above applies the target's own conversion;
 * using the register directly here would make every delay 16x too short.
 *
 * BOUNDS. Busy-waiting for milliseconds is a mistake at the call site, not
 * something this function should hide, so the loop is capped and says so once.
 * Anything longer belongs in rt_thread_mdelay(), which yields.
 */

/* Above this, the caller wanted a sleep rather than a delay. 10 ms of spinning on
 * a 320 MHz core is already ~3.2 million cycles. */
#define S31_US_DELAY_MAX    10000u

/* Above this, the caller wanted a sleep rather than a delay. 10 ms of spinning on
 * a 320 MHz core is already ~3.2 million cycles. */
#define S31_US_DELAY_MAX    10000u

void rt_hw_us_delay(rt_uint32_t us)
{
    static rt_bool_t warned;
    rt_uint64_t start;

    if (us == 0)
    {
        return;
    }

    if (us > S31_US_DELAY_MAX)
    {
        if (!warned)
        {
            warned = RT_TRUE;
            rt_kprintf("rt_hw_us_delay: %u us capped at %u us - use "
                       "rt_thread_mdelay() for anything longer\n",
                       (unsigned)us, (unsigned)S31_US_DELAY_MAX);
        }
        us = S31_US_DELAY_MAX;
    }

    start = s31_systimer_us();

    /* No yield and no interrupt enable on purpose: this has to be callable from an
     * ISR and from early board code. `volatile` on the loop condition is not
     * needed - s31_systimer_us() reads a peripheral register, which the compiler
     * cannot hoist. */
    while ((s31_systimer_us() - start) < (rt_uint64_t)us)
    {
    }
}

rt_err_t rt_hw_timer_init(void)
{
    s_group[0].hw = TIMER_LL_GET_HW(0);
    s_group[1].hw = TIMER_LL_GET_HW(1);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_timer_init);
