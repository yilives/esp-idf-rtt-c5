#ifndef PORT_USJ_H
#define PORT_USJ_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t rt_hw_usj_init(void);

/* Bounded write to the IN endpoint: returns -RT_ETIMEOUT if the FIFO never drains
 * (no host attached), rather than blocking forever. */
rt_err_t rt_hw_usj_write(const rt_uint8_t *buf, rt_uint32_t len);

/* Device-side self test. Passes without a host; says what that does not prove. */
int c5_usj_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_USJ_H */
