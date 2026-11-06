#ifndef PORT_GDMA_H
#define PORT_GDMA_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One AHB group, three channel pairs, but only TX channel 0 is brought up. */
#define C5_GDMA_GROUP   0
#define C5_GDMA_TX_CH   0
#define C5_GDMA_RX_CH   0

/* M2M trigger id, from the channel number: ch0 -> 10 (ahb_dma_ll.h documents
 * this, and AHB_DMA_LL_M2M_FREE_PERIPH_ID_MASK is 0x1C00 = bits 10..12). */
#define C5_GDMA_M2M_TRIG_ID  10

rt_err_t rt_hw_gdma_init(void);

/* Memory-to-memory copy through the TX channel, verified byte for byte.
 * Returns -RT_ETIMEOUT when the descriptor never comes back (channel did not
 * run), -RT_ERROR when it ran but the data or status is wrong. */
rt_err_t rt_hw_gdma_memcpy(void *dst, const void *src, rt_uint32_t len,
                           rt_uint32_t timeout_ms);

int c5_gdma_selftest(rt_uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* PORT_GDMA_H */

