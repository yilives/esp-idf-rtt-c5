#ifndef PORT_CACHE_H
#define PORT_CACHE_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Flush the CPU's writes out to physical memory, so a DMA reading them sees what
 * the CPU wrote. Returns 0 on success. */
int rt_hw_cache_flush(const void *addr, rt_uint32_t len);

/* Throw away the cache's copy of a region, so the CPU re-reads what a DMA wrote.
 * Returns 0 on success. */
int rt_hw_cache_invalidate(void *addr, rt_uint32_t len);

/* GDMA mem-to-mem with the cache handled for whichever side is in PSRAM. */
rt_err_t rt_hw_dma_memcpy_cached(void *dst, const void *src, rt_uint32_t len);

/* DMA in and out of PSRAM, plus the negative control that proves the cache
 * handling is what makes it work. */
int c5_dma_psram_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_CACHE_H */
