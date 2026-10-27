/*
 * General-purpose timer channels (TIMG).
 *
 * RT-Thread 5.x dropped the hwtimer device class, so this is a direct API rather
 * than a framework registration. Four channels: TIMG0 t0/t1 and TIMG1 t0/t1.
 *
 *   s31_timer_start(idx, freq_hz, cb, arg);    periodic, cb runs in the ISR
 *   s31_timer_stop(idx);
 *   s31_timer_count(idx);                      raw 1 MHz counter
 *
 * The callback runs in interrupt context: it must not block, and it should do
 * as little as possible. That is the whole point of a hardware timer - the
 * callback timing does not depend on the tick - but it also means the usual
 * RT-Thread ISR rules apply.
 */

#ifndef __PORT_TIMER_H__
#define __PORT_TIMER_H__

#include <rtthread.h>

#define S31_TIMER_NUM       4

/** Called from the timer ISR. Do not block inside. */
typedef void (*s31_timer_cb_t)(void *arg);

/**
 * Start a channel.
 *
 * @param idx     0..S31_TIMER_NUM-1
 * @param freq_hz 1 .. 1000000 (the timebase is 1 MHz)
 * @param cb      periodic callback, runs in the ISR
 * @param arg     passed to @p cb
 */
rt_err_t s31_timer_start(rt_uint32_t idx, rt_uint32_t freq_hz,
                         s31_timer_cb_t cb, void *arg);

rt_err_t s31_timer_stop(rt_uint32_t idx);

/** Raw counter, one count per microsecond. */
rt_uint64_t s31_timer_count(rt_uint32_t idx);

/**
 * SYSTIMER microseconds.
 *
 * Used to measure the timers above: TIMG and SYSTIMER are separate peripherals,
 * so this is an independent reference rather than the timer grading itself.
 */
rt_uint64_t s31_systimer_us(void);

/** Cache the TIMG instances. Called from rt_hw_board_init(). */
rt_err_t rt_hw_timer_init(void);

#endif /* __PORT_TIMER_H__ */
