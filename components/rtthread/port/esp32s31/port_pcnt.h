/*
 * PCNT (pulse counter) on the S31.
 *
 * RT-Thread does not model pulse counters, so there is no device class to plug
 * into and no `rt_pcnt_*` API to satisfy. This is therefore a plain C API plus
 * an msh command, which is the honest shape for the peripheral - inventing an
 * `rt_device` wrapper with one caller would add indirection without adding a
 * contract.
 *
 * Two groups (PCNT0/PCNT1), 4 units each, 2 signal channels + 1 control channel
 * per unit. `s31pcnt` uses group 0.
 *
 * The counter is a signed 16-bit value; `rt_hw_pcnt_get()` returns it already
 * sign-extended, so a wrap at 32767 is hardware behaviour, not a driver bug.
 *
 * Verification without wiring: a unit counts edges on a pad, and the pad's input
 * path sees whatever drives it - including this port's own GPIO output. The msh
 * command therefore drives N rising edges from software and asserts that the
 * counter reads exactly N.
 */

#ifndef __PORT_PCNT_H__
#define __PORT_PCNT_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Enable the group's clock and reset its registers. Call once. */
rt_err_t rt_hw_pcnt_init(void);

/**
 * Count positive edges on `pin` with `unit`.
 *
 * Configures the pad for GPIO input, routes it to the unit's signal channel 0,
 * and starts counting. The control channel is left at its reset action, so a
 * single pin is enough.
 *
 * @param unit 0..3 (group 0)
 * @param pin  any pad inside SOC_GPIO_VALID_GPIO_MASK
 */
rt_err_t rt_hw_pcnt_setup(int unit, rt_base_t pin);

/** Current count, sign-extended from the hardware's 16-bit field. */
int rt_hw_pcnt_get(int unit);

/** Zero the counter without stopping it. */
void rt_hw_pcnt_clear(int unit);

/**
 * Stop counting and leave the unit idle.
 *
 * Needed because a stopped counter keeps its last value and a running one
 * keeps consuming its input signal; a measurement that shares its pin with
 * another peripheral has to release the unit first.
 */
void rt_hw_pcnt_stop(int unit);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_PCNT_H__ */
