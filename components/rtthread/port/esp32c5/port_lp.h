#ifndef PORT_LP_H
#define PORT_LP_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* True when the RTC marker from a previous deep sleep is present, i.e. this boot
 * followed a sleep rather than a cold start. */
rt_bool_t rt_hw_lp_have_sleep_record(void);

/* Print what crossed the sleep: wake cause, count, and whether PSRAM retained
 * its contents. No output on a cold boot. */
void rt_hw_lp_print_sleep_record(void);

/* Arm the RTC timer and enter deep sleep. Never returns; the result appears on
 * the next boot through rt_hw_lp_print_sleep_record(). */
int c5_lp_sleep_selftest(rt_uint32_t seconds);

#ifdef __cplusplus
}
#endif

#endif /* PORT_LP_H */
