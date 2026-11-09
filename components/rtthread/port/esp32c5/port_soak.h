#ifndef PORT_SOAK_H
#define PORT_SOAK_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Run `rounds` rounds of allocate/free plus thread and semaphore create/delete,
 * sampling both heaps and the tick after each. Fails if either heap drifts past
 * its tolerance or the tick drifts from the requested sleeps.
 *
 * This is a KERNEL soak. It does not exercise Wi-Fi, BLE or any peripheral. */
int c5_soak_selftest(rt_uint32_t rounds, rt_uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* PORT_SOAK_H */
