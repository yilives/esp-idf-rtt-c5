/*
 * General-purpose timers (TIMG) with a periodic callback.
 *
 * RT-Thread 5.x has no hardware-timer device class any more - only the
 * documentation under documentation/6.components/device-driver/hwtimer survives,
 * the code is gone. So this exposes a small direct API instead of inventing a
 * device class that nothing else understands. The intended users are the kind
 * that need a deadline finer than the 1 ms tick: a bit-banged protocol, a
 * sampling loop, or jitter measurement.
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

    /* SYSTIMER unit 0 counts at 1 MHz. Read the halves with the usual
     * high-low-high guard so a carry between the two reads cannot slip in. */
    do
    {
        hi  = SYSTIMER.unit_val[0].hi.timer_unit_value_hi;
        lo  = SYSTIMER.unit_val[0].lo.timer_unit_value_lo;
        hi2 = SYSTIMER.unit_val[0].hi.timer_unit_value_hi;
    } while (hi != hi2);

    return ((rt_uint64_t)hi << 32) | lo;
}

static void s31_timg_isr(void *arg)
{
    struct s31_timg_group *g = (struct s31_timg_group *)arg;
    uint32_t status = timer_ll_get_intr_status(g->hw);

    /* Clear only the alarm bits this group actually latched. Writing back the
     * whole status register would also acknowledge bits belonging to the other
     * timer, whose handler then finds nothing pending. */
    uint32_t pending = status & (TIMER_LL_EVENT_ALARM(0) | TIMER_LL_EVENT_ALARM(1));

    if (pending == 0)
    {
        return;
    }

    timer_ll_clear_intr_status(g->hw, pending);

    for (int t = 0; t < S31_TIMG_TIMERS; t++)
    {
        if ((pending & TIMER_LL_EVENT_ALARM(t)) && g->cb[t] != RT_NULL)
        {
            g->cb[t](g->arg[t]);
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

    alarm = S31_TIMG_BASE_HZ / freq_hz;
    if (alarm > 0)
    {
        alarm -= 1;                 /* the counter hits 0 too, so subtract one */
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
        int ok = (measured > hz - (hz / 100)) && (measured < hz + (hz / 100));

        rt_kprintf("s31timer: timer %d want %d Hz, got %d Hz over %d us, %d ticks -> %s\n",
                   (int)idx, (int)hz, (int)measured, (int)elapsed_us, (int)n,
                   ok ? "PASS" : "FAIL (outside 1%)");
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_timer, s31timer, GPTimer periodic interrupt rate check);

rt_err_t rt_hw_timer_init(void)
{
    s_group[0].hw = TIMER_LL_GET_HW(0);
    s_group[1].hw = TIMER_LL_GET_HW(1);

    return RT_EOK;
}
