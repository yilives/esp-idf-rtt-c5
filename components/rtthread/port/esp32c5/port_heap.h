#ifndef PORT_HEAP_H
#define PORT_HEAP_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Allocate across four orders of magnitude and report which pool served each,
 * checked by address and by content. Fails if small allocations land in PSRAM or
 * large ones do not. */
int c5_heap_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_HEAP_H */
