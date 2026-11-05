/*
 * ESP32-S31 interrupt routing helper.
 *
 * Every INTERRUPT_CORE0_<peripheral>_MAP register resets to 0, so out of reset
 * the whole interrupt matrix points at CPU interrupt line 0. Installing a
 * handler on a line without routing the source onto it does not fail: the
 * handler is simply never reached, the peripheral's pending bit stays set, and
 * the driver looks dead. That is why routing is a separate, explicit step and
 * every driver here calls it before rt_hw_interrupt_install().
 *
 * Numbering reminder (see the porting plan, section P2.1):
 *   source  = ETS_<peripheral>_INTR_SOURCE  - matrix input index
 *   inum    = ETS_<peripheral>_INUM         - CPU interrupt line, 0..31
 *   mtvt    = inum + 16                     - CLIC vector slot, never passed here
 */

#ifndef __PORT_INTR_H__
#define __PORT_INTR_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Route a peripheral interrupt source onto a CPU interrupt line.
 *
 * Wraps esp_rom_route_intr_matrix(cpu0, source, inum). Safe to call more than
 * once with the same arguments.
 *
 * @param source ETS_*_INTR_SOURCE (interrupt matrix input index)
 * @param inum   ETS_*_INUM (CPU interrupt line, 0..SOC_CPU_INTR_NUM-1)
 * @return 0 on success, -RT_EINVAL if either number is out of range
 */
int rt_hw_intr_route(int source, int inum);

/**
 * Set the CLIC level of a CPU interrupt line.
 *
 * rt_hw_interrupt_install() gives a line level 1 the first time a handler is
 * attached, which is the weakest level the CLIC threshold lets through. Two
 * lines need different levels to nest, so a driver that wants to be preemptible
 * by a higher priority source must raise it here. Do this BEFORE
 * rt_hw_interrupt_install(), which only fills in a level when the line is still
 * at 0.
 *
 * @param vector   ETS INUM
 * @param priority CLIC level, 1..(2^NLBITS - 1); 0 means "masked"
 */
void rt_hw_interrupt_set_priority(int vector, rt_uint32_t priority);

/**
 * Remove a handler installed with rt_hw_interrupt_install().
 *
 * Only clears the slot when the pair matches, so an unrelated owner of the same
 * line is left alone. The line keeps its level and mask state; mask it
 * explicitly if the interrupt must stop arriving.
 */
void rt_hw_interrupt_uninstall(int vector, rt_isr_handler_t handler, void *param);

/**
 * Whether a CPU line's CLIC enable bit is currently clear.
 *
 * Needed because the obvious way to ask - esprv_int_get_priority() == 0 - reads
 * the *level*, not the enable bit, and the two are different: a line can be
 * enabled at level 0 (which the threshold then masks) or enabled at level 1 and
 * separately disabled through CLIC INTIE. Only INTIE is what this port writes.
 */
rt_bool_t rt_hw_interrupt_is_masked(int vector);

/**
 * How many times the trampoline has entered a line's handler.
 *
 * Diagnostic only: a counter that keeps climbing while no thread runs is the
 * signature of an interrupt storm, and the watchdog dump alone cannot name the
 * line. Returns 0 for an out-of-range vector.
 */
rt_uint32_t rt_hw_interrupt_entry_count(int vector);

/**
 * The one function with the signature esp_intr_alloc()/intr_handler_t expects.
 *
 * Drivers that allocate a line through the IDF allocator (esp_intr_alloc) must
 * pass this, with a struct rtt_isr_ctx *-shaped argument obtained from
 * rt_hw_interrupt_install + intr_handler_get_arg. In practice they call
 * rt_hw_interrupt_install(n, fn, arg, name) first and then
 * esp_intr_alloc(source, flags, rtt_trampoline, <that same line's ctx>, &h).
 *
 * Exported because port/esp32s31/gpio.c needs exactly that: the line number has
 * to come from the allocator (a hardcoded one collides with the OS tick - see
 * the note in gpio.c), while the handler still has to look like an RT-Thread
 * ISR to the rest of the port.
 */
/* No IRAM_ATTR here: it expands to a __COUNTER__-suffixed section name, so
 * repeating it on the declaration gives a conflicting section attribute. The
 * definition in interrupt.c carries it. */
void rtt_trampoline(void *arg);

/**
 * Mask every CPU line that is enabled but has no handler.
 *
 * An enabled line with an empty handler slot is not harmless: CLIC lines are
 * level-triggered by reset and _global_interrupt_handler() silently returns when
 * the slot is empty, so the trap re-enters forever and the only witness is the
 * interrupt watchdog. This was the root cause of the PSRAM bring-up crash.
 *
 * Call once, after every init stage that can allocate an interrupt (i.e. at the
 * end of board init, not before the SECONDARY-stage IDF inits have run).
 *
 * @return number of lines masked
 */
int rt_hw_intr_mask_unhandled(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_INTR_H__ */
