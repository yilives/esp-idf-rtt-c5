#ifndef PORT_RES_H
#define PORT_RES_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Report the pin map: what each pad is for, which pads are claimed twice, and
 * which test pins sit on a pad that already has a fixed function.
 *
 * Returns 0 when nothing is wrong, -1 otherwise, so it can be used as a boot
 * self-check rather than only read by a human. */
int c5_pins_report(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_RES_H */
