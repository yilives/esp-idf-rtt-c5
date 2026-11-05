#ifndef PORT_PSRAM_H
#define PORT_PSRAM_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Is the PSRAM up, and how big does it claim to be. */
rt_bool_t   rt_hw_psram_ready(void);
rt_uint32_t rt_hw_psram_size(void);

/* Report state; with test_kb > 0 also allocate from PSRAM and check it. */
int c5_psram_selftest(rt_uint32_t test_kb);

#ifdef __cplusplus
}
#endif

#endif /* PORT_PSRAM_H */
