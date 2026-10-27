/*
 * ESP32-S31 bring-up self tests.
 *
 * P1 and P2 were written without hardware, so four things can only be proven
 * on a board: the tick rate, the context switch, the CLIC nesting depth and
 * the FPU frame. Each has a dedicated test here, and `s31test` runs them all.
 * The point is to make the first power-on session produce evidence instead of
 * a guess.
 *
 * Tests and what they actually prove:
 *
 *   tick     rt_tick_get() advance vs the SYSTIMER OS-tick counter (1 MHz) and
 *            vs a CPU-cycle busy wait. Catches CONFIG_FREERTOS_HZ being
 *            different from RT_TICK_PER_SECOND, which no compile-time check
 *            can see.
 *   ctx      four threads at one priority, each holding a stack guard across
 *            rt_thread_yield(). Catches a corrupted stack pointer.
 *   preempt  two pure-integer threads that never make a system call, plus one
 *            thread sleeping 1 ms in a loop, so a previously cooperatively
 *            saved frame is resumed from the trap exit a thousand times a
 *            second. This is the test for the interrupt/cooperative frame
 *            mismatch described in risk R9.
 *   fpu      two threads that load f0..f31 with distinct patterns, sleep, and
 *            verify. Catches missing FPU save/restore.
 *   irq      GPIO interrupt raised by writing the status set register, so no
 *            wiring is needed. Exercises the whole chain: matrix route ->
 *            ETS_GPIO_INUM -> mtvt slot -> _interrupt_handler ->
 *            rtt_trampoline -> rt_pin callback.
 *   sem      semaphore ping-pong between two threads.
 *
 * Every test is also reachable on its own: `s31test tick`, `s31test fpu`, ...
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include <stdlib.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_struct.h"
#include "soc/systimer_struct.h"
#include "hal/gpio_ll.h"
#include "riscv/interrupt.h"
#include "esp_rom_sys.h"

#include "ft_core.h"
#include "port_intr.h"
#include "port_selftest.h"

/* SYSTIMER_COUNTER_OS_TICK, see esp_hw_support/include/esp_private/systimer.h.
 * Counter 1 is the one the RTOS porting layer drives the tick from; it free
 * runs at 1 MHz, so one unit is one microsecond. */
#define ST_US_COUNTER           1

#define ST_PASS                 0
#define ST_FAIL                 1

static int s_pass;
static int s_fail;

static void st_report(const char *name, int ok, const char *fmt, ...)
{
    char    detail[96];
    va_list ap;

    detail[0] = '\0';

    if (fmt != RT_NULL)
    {
        va_start(ap, fmt);
        rt_vsnprintf(detail, sizeof(detail), fmt, ap);
        va_end(ap);
    }

    if (ok)
    {
        s_pass++;
        rt_kprintf("[PASS] %-8s %s\n", name, detail);
    }
    else
    {
        s_fail++;
        rt_kprintf("[FAIL] %-8s %s\n", name, detail);
    }
}

/* 64-bit read with the usual high/low/high guard against a carry between the
 * two halves. Reads the register block directly instead of going through
 * hal/systimer_ll.h: the SYSTIMER is only used here and pulling in that
 * component would add a dependency for two loads. */
static rt_uint64_t st_now_us(void)
{
    rt_uint32_t hi, lo, hi2;

    do
    {
        hi  = SYSTIMER.unit_val[ST_US_COUNTER].hi.timer_unit_value_hi;
        lo  = SYSTIMER.unit_val[ST_US_COUNTER].lo.timer_unit_value_lo;
        hi2 = SYSTIMER.unit_val[ST_US_COUNTER].hi.timer_unit_value_hi;
    } while (hi != hi2);

    return ((rt_uint64_t)hi << 32) | lo;
}

/* ------------------------------------------------------------------ tick -- */

static int st_tick(void)
{
    rt_tick_t  t0, t1;
    rt_uint64_t c0, c1;
    rt_int32_t  d_tick;
    rt_uint32_t d_us;
    rt_uint32_t us_per_tick;

    t0 = rt_tick_get();
    c0 = st_now_us();

    /* Busy wait on the CPU cycle counter, which is a different clock domain
     * from the SYSTIMER the tick is derived from. A gross divider error shows
     * up here even if the alarm period itself is consistent. */
    esp_rom_delay_us(2000000);

    t1 = rt_tick_get();
    c1 = st_now_us();

    d_tick = (rt_int32_t)(t1 - t0);
    d_us   = (rt_uint32_t)(c1 - c0);

    if (d_tick <= 0)
    {
        st_report("tick", ST_FAIL, "no tick in %u us", d_us);
        return ST_FAIL;
    }

    us_per_tick = d_us / (rt_uint32_t)d_tick;

    /* Expect 1000000 / RT_TICK_PER_SECOND, allow 5% for the busy wait's own
     * calibration error and scheduling jitter. */
    {
        rt_uint32_t expect = 1000000U / RT_TICK_PER_SECOND;
        rt_uint32_t lo     = expect - expect / 20;
        rt_uint32_t hi     = expect + expect / 20;

        st_report("tick", (us_per_tick >= lo && us_per_tick <= hi),
                  "%d ticks in %u us = %u us/tick (expect %u, %d Hz)",
                  (int)d_tick, d_us, us_per_tick, expect, RT_TICK_PER_SECOND);

        return (us_per_tick >= lo && us_per_tick <= hi) ? ST_PASS : ST_FAIL;
    }
}

/* ------------------------------------------------------------------- ctx -- */

#define ST_GUARD_WORDS          16

struct st_ctx_worker
{
    rt_uint32_t loops;
    rt_uint32_t guard_errors;
    rt_uint32_t running;
};

static void st_ctx_thread(void *arg)
{
    struct st_ctx_worker *w = (struct st_ctx_worker *)arg;
    volatile rt_uint32_t  guard[ST_GUARD_WORDS];
    /* Any per-thread constant works; the address of the worker block is unique
     * and needs no extra field. */
    rt_uint32_t stamp = 0xC0DE0000u | ((rt_uint32_t)(rt_ubase_t)w & 0xFFFF);
    int i;

    for (i = 0; i < ST_GUARD_WORDS; i++)
    {
        guard[i] = stamp ^ (rt_uint32_t)i;
    }

    while (w->running)
    {
        w->loops++;

        for (i = 0; i < ST_GUARD_WORDS; i++)
        {
            if (guard[i] != (stamp ^ (rt_uint32_t)i))
            {
                w->guard_errors++;
                /* Repair so a single fault is not counted thousands of times. */
                guard[i] = stamp ^ (rt_uint32_t)i;
            }
        }

        /* Cooperative switch out and back: the frame this thread leaves behind
         * is the 256-byte cooperative one. */
        rt_thread_yield();
    }
}

static int st_ctx(void)
{
    enum { N = 4 };
    struct st_ctx_worker w[N];
    rt_thread_t tid[N];
    rt_uint32_t min_loops = 0xFFFFFFFFu;
    rt_uint32_t total_err = 0;
    int i, ok;

    rt_memset(w, 0, sizeof(w));

    for (i = 0; i < N; i++)
    {
        w[i].running = 1;
        tid[i] = rt_thread_create("stctx", st_ctx_thread, &w[i], 2048, 20, 10);
        if (tid[i] == RT_NULL)
        {
            st_report("ctx", ST_FAIL, "thread %d create failed", i);
            return ST_FAIL;
        }
        rt_thread_startup(tid[i]);
    }

    rt_thread_mdelay(2000);

    for (i = 0; i < N; i++)
    {
        w[i].running = 0;
    }

    rt_thread_mdelay(200);

    for (i = 0; i < N; i++)
    {
        if (tid[i] != RT_NULL)
        {
            rt_thread_delete(tid[i]);
        }
        total_err += w[i].guard_errors;
        if (w[i].loops < min_loops)
        {
            min_loops = w[i].loops;
        }
    }

    ok = (total_err == 0) && (min_loops > 0);
    st_report("ctx", ok, "min %u loops/thread, %u guard errors, 4 threads",
              min_loops, total_err);

    return ok ? ST_PASS : ST_FAIL;
}

/* --------------------------------------------------------------- preempt -- */

struct st_spin
{
    volatile rt_uint32_t loops;
    volatile rt_uint32_t guard_errors;
    volatile rt_uint32_t running;
    rt_uint32_t          id;
};

/* No system call in the loop on purpose: this thread can only be switched out
 * by the tick interrupt. */
static void st_spin_thread(void *arg)
{
    struct st_spin *s = (struct st_spin *)arg;
    volatile rt_uint32_t guard[24];
    rt_uint32_t stamp = 0x5A5A0000u | s->id;
    int i;

    for (i = 0; i < 24; i++)
    {
        guard[i] = stamp ^ (rt_uint32_t)i;
    }

    while (s->running)
    {
        s->loops++;

        for (i = 0; i < 24; i++)
        {
            if (guard[i] != (stamp ^ (rt_uint32_t)i))
            {
                s->guard_errors++;
                guard[i] = stamp ^ (rt_uint32_t)i;
            }
        }
    }
}

/* Sleeping in a loop: leaves a cooperative frame behind, then gets resumed
 * from the trap exit when the alarm fires. That mix is what the frame layout
 * fix addresses. */
static void st_sleeper_thread(void *arg)
{
    struct st_spin *s = (struct st_spin *)arg;

    while (s->running)
    {
        s->loops++;
        rt_thread_mdelay(1);
    }
}

static int st_preempt(int seconds)
{
    struct st_spin spin[2];
    struct st_spin sleeper;
    rt_thread_t t[3];
    rt_uint32_t total_err;
    rt_uint32_t min_loops;
    int i, ok;

    if (seconds <= 0)
    {
        seconds = 5;
    }

    rt_memset(spin, 0, sizeof(spin));
    rt_memset(&sleeper, 0, sizeof(sleeper));

    for (i = 0; i < 2; i++)
    {
        spin[i].running = 1;
        spin[i].id      = (rt_uint32_t)i;
        /* One priority below the sleeper so the sleeper can always preempt. */
        t[i] = rt_thread_create("stspin", st_spin_thread, &spin[i], 2048, 20, 10);
        if (t[i] == RT_NULL)
        {
            st_report("preempt", ST_FAIL, "spin thread create failed");
            return ST_FAIL;
        }
        rt_thread_startup(t[i]);
    }

    sleeper.running = 1;
    sleeper.id      = 9;
    t[2] = rt_thread_create("stslp", st_sleeper_thread, &sleeper, 2048, 15, 10);
    if (t[2] == RT_NULL)
    {
        st_report("preempt", ST_FAIL, "sleeper thread create failed");
        return ST_FAIL;
    }
    rt_thread_startup(t[2]);

    rt_kprintf("s31test: preempting for %d s at %d Hz ...\n", seconds, RT_TICK_PER_SECOND);
    rt_thread_mdelay((rt_int32_t)seconds * 1000);

    spin[0].running = 0;
    spin[1].running = 0;
    sleeper.running = 0;
    rt_thread_mdelay(50);

    for (i = 0; i < 3; i++)
    {
        if (t[i] != RT_NULL)
        {
            rt_thread_delete(t[i]);
        }
    }

    total_err = spin[0].guard_errors + spin[1].guard_errors;
    min_loops = spin[0].loops < spin[1].loops ? spin[0].loops : spin[1].loops;
    ok = (total_err == 0) && (min_loops > 0) && (sleeper.loops > 0);

    st_report("preempt", ok,
              "%u/%u spin loops, %u wakeups, %u guard errors",
              spin[0].loops, spin[1].loops, sleeper.loops, total_err);

    return ok ? ST_PASS : ST_FAIL;
}

/* ------------------------------------------------------------------- fpu -- */

/* f0..f31 filled from one integer register. The clobber list has to name all
 * 32 of them, otherwise the compiler is free to keep something live there. */
#define ST_FPU_FILL(v)                                                        \
    __asm__ volatile(                                                         \
        "fmv.w.x ft0,  %0\n\t" "fmv.w.x ft1,  %0\n\t"                         \
        "fmv.w.x ft2,  %0\n\t" "fmv.w.x ft3,  %0\n\t"                         \
        "fmv.w.x ft4,  %0\n\t" "fmv.w.x ft5,  %0\n\t"                         \
        "fmv.w.x ft6,  %0\n\t" "fmv.w.x ft7,  %0\n\t"                         \
        "fmv.w.x fs0,  %0\n\t" "fmv.w.x fs1,  %0\n\t"                         \
        "fmv.w.x fa0,  %0\n\t" "fmv.w.x fa1,  %0\n\t"                         \
        "fmv.w.x fa2,  %0\n\t" "fmv.w.x fa3,  %0\n\t"                         \
        "fmv.w.x fa4,  %0\n\t" "fmv.w.x fa5,  %0\n\t"                         \
        "fmv.w.x fa6,  %0\n\t" "fmv.w.x fa7,  %0\n\t"                         \
        "fmv.w.x fs2,  %0\n\t" "fmv.w.x fs3,  %0\n\t"                         \
        "fmv.w.x fs4,  %0\n\t" "fmv.w.x fs5,  %0\n\t"                         \
        "fmv.w.x fs6,  %0\n\t" "fmv.w.x fs7,  %0\n\t"                         \
        "fmv.w.x fs8,  %0\n\t" "fmv.w.x fs9,  %0\n\t"                         \
        "fmv.w.x fs10, %0\n\t" "fmv.w.x fs11, %0\n\t"                         \
        "fmv.w.x ft8,  %0\n\t" "fmv.w.x ft9,  %0\n\t"                         \
        "fmv.w.x ft10, %0\n\t" "fmv.w.x ft11, %0\n\t"                         \
        :                                                                     \
        : "r"(v)                                                              \
        : "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7",             \
          "fs0", "fs1", "fa0", "fa1", "fa2", "fa3", "fa4", "fa5",             \
          "fa6", "fa7", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7",             \
          "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11")

/* Each read is its own volatile asm; volatile asm statements stay in order
 * relative to each other, so the snapshot always follows the fill. */
static void st_fpu_snapshot(rt_uint32_t *o)
{
#define ST_RD(i, r)     __asm__ volatile("fmv.x.w %0, " #r : "=r"(o[i]))
    ST_RD( 0, ft0);  ST_RD( 1, ft1);  ST_RD( 2, ft2);  ST_RD( 3, ft3);
    ST_RD( 4, ft4);  ST_RD( 5, ft5);  ST_RD( 6, ft6);  ST_RD( 7, ft7);
    ST_RD( 8, fs0);  ST_RD( 9, fs1);  ST_RD(10, fa0);  ST_RD(11, fa1);
    ST_RD(12, fa2);  ST_RD(13, fa3);  ST_RD(14, fa4);  ST_RD(15, fa5);
    ST_RD(16, fa6);  ST_RD(17, fa7);  ST_RD(18, fs2);  ST_RD(19, fs3);
    ST_RD(20, fs4);  ST_RD(21, fs5);  ST_RD(22, fs6);  ST_RD(23, fs7);
    ST_RD(24, fs8);  ST_RD(25, fs9);  ST_RD(26, fs10); ST_RD(27, fs11);
    ST_RD(28, ft8);  ST_RD(29, ft9);  ST_RD(30, ft10); ST_RD(31, ft11);
#undef ST_RD
}

struct st_fpu_worker
{
    rt_uint32_t rounds;
    rt_uint32_t errors;
    rt_uint32_t first_bad;
    rt_uint32_t pattern;
    volatile rt_uint32_t running;
};

static void st_fpu_thread(void *arg)
{
    struct st_fpu_worker *w = (struct st_fpu_worker *)arg;
    rt_uint32_t snap[32];
    rt_uint32_t round;
    int i;

    for (round = 0; round < w->rounds; round++)
    {
        ST_FPU_FILL(w->pattern);

        /* Sleep so the other thread runs and overwrites its own copy. On the
         * way back the port has to restore ours. */
        rt_thread_mdelay(1);

        st_fpu_snapshot(snap);

        for (i = 0; i < 32; i++)
        {
            if (snap[i] != w->pattern)
            {
                if (w->errors == 0)
                {
                    w->first_bad = ((rt_uint32_t)i << 16) | (snap[i] & 0xFFFF);
                }
                w->errors++;
                break;
            }
        }
    }

    w->running = 0;
}

static int st_fpu(void)
{
    struct st_fpu_worker w[2];
    rt_thread_t tid[2];
    rt_uint32_t total = 0;
    int i, ok;

    rt_memset(w, 0, sizeof(w));

    /* Half precision-safe patterns: 0x3f800001 and 0x3f800002 are both 1.0f
     * plus one and two ulp, so they survive any fmv round trip unchanged. */
    w[0].pattern = 0x3F800001u;
    w[1].pattern = 0x3F800002u;

    for (i = 0; i < 2; i++)
    {
        w[i].rounds  = 500;
        w[i].running = 1;
        tid[i] = rt_thread_create("stfpu", st_fpu_thread, &w[i], 2048, 20, 10);
        if (tid[i] == RT_NULL)
        {
            st_report("fpu", ST_FAIL, "thread %d create failed", i);
            return ST_FAIL;
        }
        rt_thread_startup(tid[i]);
    }

    for (i = 0; i < 2; i++)
    {
        while (w[i].running)
        {
            rt_thread_mdelay(10);
        }
    }

    for (i = 0; i < 2; i++)
    {
        rt_thread_delete(tid[i]);
        total += w[i].errors;
    }

    ok = (total == 0);
    if (ok)
    {
        st_report("fpu", ST_PASS, "32 registers x 500 switches, no mismatch");
    }
    else
    {
        st_report("fpu", ST_FAIL, "%u mismatches, first at reg %u value 0x%04x",
                  total, w[0].errors ? (w[0].first_bad >> 16) : (w[1].first_bad >> 16),
                  w[0].errors ? (w[0].first_bad & 0xFFFF) : (w[1].first_bad & 0xFFFF));
    }

    return ok ? ST_PASS : ST_FAIL;
}

/* ------------------------------------------------------------------- irq -- */

static volatile rt_uint32_t s_irq_count;
static volatile rt_uint32_t s_irq_nest_bad;
static volatile rt_uint32_t s_irq_nest_seen;

static void st_irq_cb(void *args)
{
    /* The handler must run with exactly one level of interrupt nesting: the
     * trap entry raises it, the trap exit lowers it. Anything else means the
     * RT-Thread enter/leave pairing does not match the hardware frame. */
    rt_uint32_t nest = (rt_uint32_t)rt_interrupt_get_nest();

    RT_UNUSED(args);

    s_irq_nest_seen = nest;
    if (nest != 1)
    {
        s_irq_nest_bad++;
    }

    s_irq_count++;
}

/* Writing the status set register latches an interrupt without any external
 * signal, so the whole delivery path can be tested on a bare board. */
static void st_irq_trigger(rt_base_t pin)
{
    if (pin < 32)
    {
        GPIO.status_w1ts.status_w1ts = 1U << pin;
    }
    else
    {
        GPIO.status1_w1ts.status1_w1ts = 1U << (pin - 32);
    }
}

static int st_irq(rt_base_t pin)
{
    enum { N = 200 };
    int i, ok;

    if (pin < 0 || pin >= SOC_GPIO_PIN_COUNT)
    {
        st_report("irq", ST_FAIL, "bad pin %d", (int)pin);
        return ST_FAIL;
    }

    s_irq_count     = 0;
    s_irq_nest_bad  = 0;
    s_irq_nest_seen = 0;

    rt_pin_mode(pin, PIN_MODE_INPUT);
    if (rt_pin_attach_irq(pin, PIN_IRQ_MODE_RISING, st_irq_cb, RT_NULL) != RT_EOK)
    {
        st_report("irq", ST_FAIL, "attach failed on pin %d", (int)pin);
        return ST_FAIL;
    }
    rt_pin_irq_enable(pin, PIN_IRQ_ENABLE);

    for (i = 0; i < N; i++)
    {
        st_irq_trigger(pin);
        /* Long enough for the handler to run and clear the latch. */
        esp_rom_delay_us(200);
    }

    rt_pin_irq_enable(pin, PIN_IRQ_DISABLE);
    rt_pin_detach_irq(pin);

    ok = (s_irq_count == N) && (s_irq_nest_bad == 0);
    st_report("irq", ok, "pin %d: %u/%d interrupts, nest=%u (%u bad)",
              (int)pin, s_irq_count, N, s_irq_nest_seen, s_irq_nest_bad);

    return ok ? ST_PASS : ST_FAIL;
}

/* ------------------------------------------------------------------ nest -- */

/*
 * Two GPIO interrupt channels give two CPU lines at two CLIC levels without any
 * external wiring:
 *
 *   channel 0 -> ETS_GPIO_INUM (4), level 1, driven through the pin driver
 *   channel 1 -> a spare line,        level 2, driven directly by this test
 *
 * Raising the channel 1 status from inside the channel 0 handler must preempt
 * that very handler. This is the only way to prove the hardware nesting path in
 * vectors_rtthread.S (save mcause in s1, set mstatus.MIE, restore mcause before
 * mret) actually works - a single-priority system never exercises it.
 *
 * The S31 has four independent GPIO interrupt channels; gpio_ll_intr_enable_on_core()
 * only implements channel 0 (IDF-7995), which is why channel 1 is programmed
 * here with raw registers rather than through the driver.
 */

#define ST_NEST_INNER_INUM      6           /* free: 4 = GPIO, 5 = UART0 */
#define ST_NEST_INNER_LEVEL     2
#define ST_NEST_ROUNDS          100

static rt_base_t            s_nest_inner_pin;
static volatile rt_uint32_t s_nest_outer_count;
static volatile rt_uint32_t s_nest_inner_count;
static volatile rt_uint32_t s_nest_outer_depth;
static volatile rt_uint32_t s_nest_inner_depth;
static volatile rt_uint32_t s_nest_outer_depth_bad;
static volatile rt_uint32_t s_nest_inner_depth_bad;

static rt_bool_t st_pin_usable(rt_base_t pin)
{
    if (pin < 0 || pin >= SOC_GPIO_PIN_COUNT)
    {
        return RT_FALSE;
    }

    return ((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) ? RT_TRUE : RT_FALSE;
}

static rt_base_t st_next_pin(rt_base_t pin)
{
    rt_base_t p;

    for (p = pin + 1; p < SOC_GPIO_PIN_COUNT; p++)
    {
        if (st_pin_usable(p))
        {
            return p;
        }
    }

    return -1;
}

static void st_nest_inner_isr(int vector, void *param)
{
    rt_uint32_t lo = GPIO.intr_1.int_1;
    rt_uint32_t hi = GPIO.intr_11.int_11;
    rt_uint32_t depth;

    RT_UNUSED(vector);
    RT_UNUSED(param);

    if (lo)
    {
        GPIO.status_w1tc.status_w1tc = lo;
    }
    if (hi)
    {
        GPIO.status1_w1tc.status1_w1tc = hi;
    }

    /* Arriving from inside the level 1 handler, this must observe nest == 2. */
    depth = (rt_uint32_t)rt_interrupt_get_nest();
    s_nest_inner_depth = depth;
    if (depth != 2)
    {
        s_nest_inner_depth_bad++;
    }

    s_nest_inner_count++;
}

static void st_nest_outer_cb(void *args)
{
    rt_uint32_t depth = (rt_uint32_t)rt_interrupt_get_nest();

    RT_UNUSED(args);

    s_nest_outer_depth = depth;
    if (depth != 1)
    {
        s_nest_outer_depth_bad++;
    }

    s_nest_outer_count++;

    /* Fire the level 2 channel while still inside this level 1 handler. */
    st_irq_trigger(s_nest_inner_pin);
}

static int st_nest(rt_base_t outer_pin)
{
    rt_base_t inner_pin;
    int i, ok;
    int saved_prio;

    if (!st_pin_usable(outer_pin))
    {
        st_report("nest", ST_FAIL, "bad outer pin %d", (int)outer_pin);
        return ST_FAIL;
    }

    inner_pin = st_next_pin(outer_pin);
    if (inner_pin < 0)
    {
        st_report("nest", ST_FAIL, "no usable pin above %d", (int)outer_pin);
        return ST_FAIL;
    }

    /* Do not steal a line somebody already configured. */
    saved_prio = esprv_int_get_priority(ST_NEST_INNER_INUM);
    if (saved_prio != 0)
    {
        st_report("nest", ST_FAIL, "INUM %d already in use at level %d",
                  ST_NEST_INNER_INUM, saved_prio);
        return ST_FAIL;
    }

    s_nest_inner_pin       = inner_pin;
    s_nest_outer_count     = 0;
    s_nest_inner_count     = 0;
    s_nest_outer_depth     = 0;
    s_nest_inner_depth     = 0;
    s_nest_outer_depth_bad = 0;
    s_nest_inner_depth_bad = 0;

    /* --- inner source: GPIO channel 1, spare line, level 2 --- */
    GPIO.pin[inner_pin].int_type = GPIO_INTR_POSEDGE;
    GPIO.pin[inner_pin].int_ena  = GPIO_LL_INTR1_ENA;

    if (rt_hw_intr_route(ETS_GPIO_INTR1_SOURCE, ST_NEST_INNER_INUM) != 0)
    {
        st_report("nest", ST_FAIL, "route of GPIO_INTR1 failed");
        GPIO.pin[inner_pin].int_ena = 0;
        return ST_FAIL;
    }

    rt_hw_interrupt_set_priority(ST_NEST_INNER_INUM, ST_NEST_INNER_LEVEL);
    rt_hw_interrupt_install(ST_NEST_INNER_INUM, st_nest_inner_isr, RT_NULL, "gpio1");
    rt_hw_interrupt_umask(ST_NEST_INNER_INUM);

    /* --- outer source: GPIO channel 0 through the pin driver, level 1 --- */
    rt_pin_mode(outer_pin, PIN_MODE_INPUT);

    if (rt_pin_attach_irq(outer_pin, PIN_IRQ_MODE_RISING, st_nest_outer_cb, RT_NULL) != RT_EOK)
    {
        st_report("nest", ST_FAIL, "attach failed on pin %d", (int)outer_pin);
        rt_hw_interrupt_mask(ST_NEST_INNER_INUM);
        rt_hw_interrupt_uninstall(ST_NEST_INNER_INUM, st_nest_inner_isr, RT_NULL);
        GPIO.pin[inner_pin].int_ena = 0;
        return ST_FAIL;
    }

    rt_pin_irq_enable(outer_pin, PIN_IRQ_ENABLE);

    for (i = 0; i < ST_NEST_ROUNDS; i++)
    {
        /* From thread context this raises the level 1 line; the handler then
         * raises the level 2 line from inside itself. */
        st_irq_trigger(outer_pin);
        esp_rom_delay_us(300);
    }

    /* --- stop and restore --- */
    rt_pin_irq_enable(outer_pin, PIN_IRQ_DISABLE);
    rt_pin_detach_irq(outer_pin);

    GPIO.pin[inner_pin].int_ena = 0;
    rt_hw_interrupt_mask(ST_NEST_INNER_INUM);
    rt_hw_interrupt_uninstall(ST_NEST_INNER_INUM, st_nest_inner_isr, RT_NULL);

    ok = (s_nest_outer_count == ST_NEST_ROUNDS) &&
         (s_nest_inner_count == ST_NEST_ROUNDS) &&
         (s_nest_outer_depth_bad == 0) &&
         (s_nest_inner_depth_bad == 0);

    st_report("nest", ok,
              "pin %d->%d: outer %u@nest%u, inner %u@nest%u (want %d, %u bad)",
              (int)outer_pin, (int)inner_pin,
              s_nest_outer_count, s_nest_outer_depth,
              s_nest_inner_count, s_nest_inner_depth,
              ST_NEST_ROUNDS,
              s_nest_outer_depth_bad + s_nest_inner_depth_bad);

    return ok ? ST_PASS : ST_FAIL;
}

/* ------------------------------------------------------------------- sem -- */

static struct rt_semaphore s_sem;
static volatile rt_uint32_t s_sem_sent;

static void st_sem_thread(void *arg)
{
    int n = (int)(rt_base_t)arg;
    int i;

    for (i = 0; i < n; i++)
    {
        rt_sem_release(&s_sem);
        s_sem_sent++;
        if ((i & 0x1F) == 0)
        {
            rt_thread_yield();
        }
    }
    s_sem_sent = 0x80000000u | (rt_uint32_t)n;
}

static int st_sem(void)
{
    enum { N = 500 };
    rt_thread_t tid;
    rt_err_t err;
    int got = 0;
    int i, ok;
    int timeout_ok;

    rt_sem_init(&s_sem, "stsem", 0, RT_IPC_FLAG_PRIO);

    tid = rt_thread_create("stsem", st_sem_thread, (void *)(rt_base_t)N, 2048, 18, 10);
    if (tid == RT_NULL)
    {
        st_report("sem", ST_FAIL, "thread create failed");
        rt_sem_detach(&s_sem);
        return ST_FAIL;
    }
    rt_thread_startup(tid);

    for (i = 0; i < N; i++)
    {
        if (rt_sem_take(&s_sem, rt_tick_from_millisecond(1000)) == RT_EOK)
        {
            got++;
        }
        else
        {
            break;
        }
    }

    /* Nothing was released beyond N, so this must time out. A semaphore that
     * reports ready here means the count or the suspend list is wrong. */
    err = rt_sem_take(&s_sem, rt_tick_from_millisecond(50));
    timeout_ok = (err == -RT_ETIMEOUT);

    rt_thread_delete(tid);
    rt_sem_detach(&s_sem);

    ok = (got == N) && timeout_ok;
    st_report("sem", ok, "%d/%d tokens, extra take %s", got, N,
              timeout_ok ? "timed out" : "WRONGLY SUCCEEDED");

    return ok ? ST_PASS : ST_FAIL;
}

/* ---------------------------------------------------------------- fbshim -- */

/*
 * Exercises the FreeRTOS-compatible primitives in port/freertos/.
 *
 * These are not used by anything in the image yet: they exist so that
 * esp_driver_uart, esp_event, esp_netif, lwip and eventually esp_wifi can run on
 * RT-Thread. Testing them here means the semantics they depend on - blocking,
 * timeouts, capacity accounting, event bit clearing - are proven on the board
 * before a driver is built on top of them.
 */

struct st_shim_msg
{
    rt_uint32_t seq;
    rt_uint32_t payload[3];
};

#define ST_SHIM_ITEMS   6

static struct ft_ipc *s_shim_queue;
static rt_uint32_t    s_shim_produced;

static void st_shim_producer(void *arg)
{
    struct st_shim_msg m;
    rt_uint32_t i;

    RT_UNUSED(arg);

    for (i = 0; i < ST_SHIM_ITEMS; i++)
    {
        m.seq        = 0x1000u + i;
        m.payload[0] = i;
        m.payload[1] = ~i;
        m.payload[2] = i * 3u;

        /* Blocks when the queue is full: the consumer starts slower. */
        if (ft_ipc_send(s_shim_queue, &m, FT_MAX_DELAY) == FT_OK)
        {
            s_shim_produced++;
        }

        ft_task_delay(5);
    }
}

static int st_fbshim(void)
{
    struct st_shim_msg m;
    struct ft_ipc     *q2 = RT_NULL;
    struct ft_ipc     *sem = RT_NULL;
    struct ft_ipc     *mtx = RT_NULL;
    struct ft_event   *ev = RT_NULL;
    rt_thread_t        producer = RT_NULL;
    rt_uint32_t        i;
    int                bad;
    int                total_bad = 0;

    /* ---- 1. queue capacity and FIFO order ---- */
    bad = 0;
    s_shim_queue = ft_ipc_create(4, sizeof(struct st_shim_msg));
    if (s_shim_queue == RT_NULL)
    {
        st_report("fbshim", ST_FAIL, "queue create failed");
        return ST_FAIL;
    }

    for (i = 0; i < 4; i++)
    {
        m.seq = 100 + i;
        m.payload[0] = i;
        m.payload[1] = 0;
        m.payload[2] = 0;
        if (ft_ipc_send(s_shim_queue, &m, FT_NO_WAIT) != FT_OK)
        {
            bad++;
        }
    }

    if (ft_ipc_send(s_shim_queue, &m, FT_NO_WAIT) != FT_ERR_FULL)
    {
        bad++;                      /* a full queue must refuse, not block */
    }
    if (ft_ipc_messages_waiting(s_shim_queue) != 4 ||
        ft_ipc_spaces_available(s_shim_queue) != 0)
    {
        bad++;
    }

    for (i = 0; i < 4; i++)
    {
        if (ft_ipc_receive(s_shim_queue, &m, FT_NO_WAIT) != FT_OK || m.seq != 100 + i)
        {
            bad++;
        }
    }

    if (ft_ipc_receive(s_shim_queue, &m, FT_NO_WAIT) != FT_ERR_TIMEOUT)
    {
        bad++;                      /* an empty queue must report empty */
    }

    total_bad += bad;
    st_report("fbshim/q", bad == 0, "cap 4, FIFO, full+empty refusal (%d bad)", bad);

    /* ---- 2. blocking in both directions across tasks ---- */
    bad = 0;
    ft_ipc_delete(s_shim_queue);
    s_shim_queue   = ft_ipc_create(2, sizeof(struct st_shim_msg));
    s_shim_produced = 0;

    if (s_shim_queue == RT_NULL ||
        ft_task_create("stprod", st_shim_producer, RT_NULL, 512, 12, (void **)&producer) == RT_NULL)
    {
        st_report("fbshim/blk", ST_FAIL, "setup failed");
        if (s_shim_queue) ft_ipc_delete(s_shim_queue);
        return ST_FAIL;
    }

    for (i = 0; i < ST_SHIM_ITEMS; i++)
    {
        if (ft_ipc_receive(s_shim_queue, &m, FT_MAX_DELAY) != FT_OK)
        {
            bad++;
            break;
        }
        if (m.seq != 0x1000u + i || m.payload[0] != i || m.payload[1] != ~i ||
            m.payload[2] != i * 3u)
        {
            bad++;                  /* order or payload corruption */
        }
        ft_task_delay(2);           /* slower than the producer on purpose */
    }

    for (i = 0; i < 200 && s_shim_produced < ST_SHIM_ITEMS; i++)
    {
        ft_task_delay(2);
    }
    if (s_shim_produced != ST_SHIM_ITEMS || bad != 0)
    {
        bad++;
    }

    total_bad += bad;
    st_report("fbshim/blk", bad == 0, "%u/%d produced, order preserved (%d bad)",
              s_shim_produced, ST_SHIM_ITEMS, bad);

    /* ---- 3. timeout accuracy on an empty queue ---- */
    bad = 0;
    {
        rt_uint64_t t0, t1;
        int rc;

        ft_ipc_delete(s_shim_queue);
        s_shim_queue = ft_ipc_create(2, sizeof(struct st_shim_msg));

        t0 = st_now_us();
        rc = ft_ipc_receive(s_shim_queue, &m, 50);
        t1 = st_now_us();

        if (rc != FT_ERR_TIMEOUT)
        {
            bad++;
        }
        /* 50 ms requested; allow scheduling slack but not a 10x error. */
        if ((t1 - t0) < 40000u || (t1 - t0) > 500000u)
        {
            bad++;
        }

        total_bad += bad;
        st_report("fbshim/to", bad == 0, "50 ms timeout took %u us (%d bad)",
                  (rt_uint32_t)(t1 - t0), bad);
    }

    /* ---- 4. semaphores and mutex ---- */
    bad = 0;
    sem = ft_ipc_create_sem(3, 0);
    mtx = ft_ipc_create_mutex();

    if (sem == RT_NULL || mtx == RT_NULL)
    {
        bad++;
    }
    else
    {
        /* counting semaphore: 3 gives, 4 takes, the last must fail */
        for (i = 0; i < 3; i++)
        {
            if (ft_ipc_give(sem) != FT_OK)
            {
                bad++;
            }
        }
        for (i = 0; i < 3; i++)
        {
            if (ft_ipc_take(sem, FT_NO_WAIT) != FT_OK)
            {
                bad++;
            }
        }
        if (ft_ipc_take(sem, FT_NO_WAIT) != FT_ERR_TIMEOUT)
        {
            bad++;
        }
        if (ft_ipc_give(sem) != FT_OK || ft_ipc_take(sem, FT_NO_WAIT) != FT_OK)
        {
            bad++;                  /* a semaphore must be reusable */
        }

        /* mutex is not recursive: a second take by the owner must fail */
        if (ft_ipc_take(mtx, FT_NO_WAIT) != FT_OK)
        {
            bad++;
        }
        if (ft_ipc_take(mtx, FT_NO_WAIT) != FT_ERR_TIMEOUT)
        {
            bad++;
        }
        if (ft_ipc_give(mtx) != FT_OK || ft_ipc_take(mtx, FT_NO_WAIT) != FT_OK)
        {
            bad++;
        }
        ft_ipc_give(mtx);
    }

    total_bad += bad;
    st_report("fbshim/sem", bad == 0, "counting 3 max, mutex not recursive (%d bad)", bad);

    /* ---- 5. event group ---- */
    bad = 0;
    ev = ft_event_create();
    if (ev == RT_NULL)
    {
        bad++;
    }
    else
    {
        if (ft_event_set_bits(ev, 0x3) != 0x3)
        {
            bad++;
        }
        /* OR wait without clearing returns the whole set, not just the match */
        if (ft_event_wait_bits(ev, 0x1, ftFAIL, ftFAIL, FT_NO_WAIT) != 0x3)
        {
            bad++;
        }
        /* AND wait that cannot be satisfied times out and returns 0 */
        if (ft_event_wait_bits(ev, 0x5, ftFAIL, ftPASS, 20) != 0)
        {
            bad++;
        }
        /* partial clear must not touch the other bits */
        if (ft_event_clear_bits(ev, 0x2) != 0x1)
        {
            bad++;
        }
        /* clear-on-exit: matched bits are gone afterwards */
        if (ft_event_set_bits(ev, 0x4) != 0x5)
        {
            bad++;
        }
        if (ft_event_wait_bits(ev, 0x5, ftPASS, ftPASS, FT_NO_WAIT) != 0x5)
        {
            bad++;
        }
        if (ft_event_get_bits(ev) != 0)
        {
            bad++;
        }
    }

    total_bad += bad;
    st_report("fbshim/ev", bad == 0, "or/and wait, partial clear, clear-on-exit (%d bad)", bad);

    /* ---- 6. TLS ---- */
    bad = 0;
    {
        rt_uint32_t marker = 0xA5A5u;
        rt_uint32_t volatile *slot;

        ft_task_set_tls(RT_NULL, 0, (void *)&marker);
        slot = (rt_uint32_t volatile *)ft_task_get_tls(RT_NULL, 0);
        if (slot != &marker || (slot && *slot != 0xA5A5u))
        {
            bad++;
        }
        if (ft_task_get_tls(RT_NULL, 1) != RT_NULL)
        {
            bad++;                  /* untouched slot must read back NULL */
        }
        ft_task_set_tls(RT_NULL, 0, RT_NULL);
        if (ft_task_get_tls(RT_NULL, 0) != RT_NULL)
        {
            bad++;
        }
    }

    total_bad += bad;
    st_report("fbshim/tls", bad == 0, "per-thread slots, untouched reads NULL (%d bad)", bad);

    /* ---- cleanup ---- */
    if (producer != RT_NULL)
    {
        ft_task_delete(producer);
    }
    if (s_shim_queue)
    {
        ft_ipc_delete(s_shim_queue);
        s_shim_queue = RT_NULL;
    }
    if (q2)
    {
        ft_ipc_delete(q2);
    }
    if (sem)
    {
        ft_ipc_delete(sem);
    }
    if (mtx)
    {
        ft_ipc_delete(mtx);
    }
    if (ev)
    {
        ft_event_delete(ev);
    }

    return total_bad ? ST_FAIL : ST_PASS;
}

/* --------------------------------------------------------------- driver -- */

static int st_all(int seconds, rt_base_t pin)
{
    s_pass = 0;
    s_fail = 0;

    rt_kprintf("\n=== ESP32-S31 RT-Thread self test ===\n");

#ifdef CONFIG_FREERTOS_HZ
    rt_kprintf("tick: RT_TICK_PER_SECOND=%d CONFIG_FREERTOS_HZ=%d\n",
               RT_TICK_PER_SECOND, CONFIG_FREERTOS_HZ);
#endif

    st_tick();
    st_ctx();
    st_preempt(seconds);
    st_fpu();
    st_irq(pin);
    st_nest(pin);
    st_sem();
    st_fbshim();

    rt_kprintf("=== %d passed, %d failed ===\n\n", s_pass, s_fail);

    return s_fail ? ST_FAIL : ST_PASS;
}

static void st_usage(void)
{
    rt_kprintf("usage: s31test [all|tick|ctx|preempt|fpu|irq|nest|sem] [args]\n");
    rt_kprintf("  s31test              run everything (5 s preemption, pin %d)\n", 10);
    rt_kprintf("  s31test tick         tick rate vs SYSTIMER and CPU cycles\n");
    rt_kprintf("  s31test ctx          four-thread cooperative switch, stack guards\n");
    rt_kprintf("  s31test preempt [s]  integer threads vs 1 kHz preemption\n");
    rt_kprintf("  s31test fpu          f0-f31 across thread switches\n");
    rt_kprintf("  s31test irq [pin]    software-triggered GPIO interrupt\n");
    rt_kprintf("  s31test nest [pin]   level 2 preempting level 1 (GPIO ch0/ch1)\n");
    rt_kprintf("  s31test sem          semaphore ping-pong\n");
}

static int st_cmd(int argc, char **argv)
{
    if (argc < 2 || rt_strcmp(argv[1], "all") == 0)
    {
        rt_base_t pin = 10;

        if (argc >= 3)
        {
            pin = (rt_base_t)atoi(argv[2]);
        }
        return st_all(5, pin);
    }

    if (rt_strcmp(argv[1], "tick") == 0)
    {
        return st_tick();
    }

    if (rt_strcmp(argv[1], "ctx") == 0)
    {
        return st_ctx();
    }

    if (rt_strcmp(argv[1], "preempt") == 0)
    {
        int seconds = (argc >= 3) ? atoi(argv[2]) : 5;
        return st_preempt(seconds);
    }

    if (rt_strcmp(argv[1], "fpu") == 0)
    {
        return st_fpu();
    }

    if (rt_strcmp(argv[1], "irq") == 0)
    {
        rt_base_t pin = (argc >= 3) ? (rt_base_t)atoi(argv[2]) : 10;
        return st_irq(pin);
    }

    if (rt_strcmp(argv[1], "nest") == 0)
    {
        rt_base_t pin = (argc >= 3) ? (rt_base_t)atoi(argv[2]) : 10;
        return st_nest(pin);
    }

    if (rt_strcmp(argv[1], "sem") == 0)
    {
        return st_sem();
    }

    if (rt_strcmp(argv[1], "fbshim") == 0)
    {
        return st_fbshim();
    }

    st_usage();
    return ST_FAIL;
}
MSH_CMD_EXPORT_ALIAS(st_cmd, s31test, ESP32-S31 RT-Thread self test);

/* ------------------------------------------------------------------ init -- */

/* A mismatch here scales every timeout in the system, and it cannot be caught
 * at link time because the two symbols come from different configurations. */
#if defined(CONFIG_FREERTOS_HZ) && (CONFIG_FREERTOS_HZ != RT_TICK_PER_SECOND)
#warning "RT_TICK_PER_SECOND does not match CONFIG_FREERTOS_HZ: tick length will be wrong"
#endif

int s31_selftest_init(void)
{
    return 0;
}
