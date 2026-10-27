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

#ifdef __cplusplus
}
#endif

#endif /* __PORT_INTR_H__ */
