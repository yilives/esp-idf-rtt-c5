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
/* Reserve margin inside the PSRAM region for the IDF heap's own metadata. */
#define C5_PSRAM_HEAP_RESERVE   (256u * 1024u)

/* Claim one large PSRAM block for RT-Thread's allocator and report its size.
 * Returns NULL when PSRAM is not up or nothing could be claimed. The block is
 * never freed: it belongs to RT-Thread from this point on. */
void *rt_hw_psram_claim(rt_uint32_t *out_size);

#if defined(RT_USING_MEMHEAP)
/* Register the claimed block as a second memheap named "psram". With
 * RT_USING_MEMHEAP_AUTO_BINDING, rt_malloc() then falls back to it when the
 * system heap cannot satisfy a request. */
rt_err_t   rt_hw_psram_heap_init(void);
rt_bool_t  rt_hw_psram_heap_ready(void);
rt_uint32_t rt_hw_psram_heap_size(void);
#endif

int c5_psram_selftest(rt_uint32_t test_kb);

#ifdef __cplusplus
}
#endif

#endif /* PORT_PSRAM_H */


