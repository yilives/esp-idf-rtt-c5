#ifndef PORT_CAN_H
#define PORT_CAN_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One controller, matching port_twai.h's C5_TWAI_ID. */
#define C5_CAN_NAME     "can0"

rt_err_t rt_hw_can_c5_init(void);

/* Drive the rt_can device class (open/write/read) and check the loopback frame. */
int c5_can_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_CAN_H */
