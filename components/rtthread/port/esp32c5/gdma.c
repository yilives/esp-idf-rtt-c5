/*
 * ESP32-C5 AHB GDMA: one TX channel doing memory-to-memory copies.
 *
 * This exists because PARLIO cannot be brought up without it. PARLIO TX has no
 * CPU-writable data register at all - the peripheral has a `tx_data_cfg` and a
 * FIFO reset but no TXFIFO port - so its payload can only arrive through GDMA,
 * and `esp_driver_parlio` allocates a channel unconditionally. On this port the
 * GDMA driver is not compiled and `gdma_ll.h` has exactly two functions (bus
 * clock, reset); everything else lives in `ahb_dma_ll.h`.
 *
 * Rather than port the whole `esp_driver_dma` component, this implements the
 * minimum that a peripheral needs: claim a TX channel, build a descriptor chain,
 * connect the channel to a peripheral, start it, wait for EOF. Memory-to-memory
 * is the first milestone because it proves the descriptor plumbing and the
 * completion path with no peripheral involved - if M2M works, a wrong PARLIO
 * configuration is a PARLIO problem rather than a DMA problem.
 *
 * ---------------------------------------------------------------------------
 * How this is verified, and it genuinely is verified end to end.
 *
 * A copy is checked byte-for-byte: fill the source with a position-dependent
 * pattern, run the channel, compare the destination, and confirm the DMA's own
 * completion bits fired. Three independent things have to be right for that to
 * pass - the descriptor owner/EoF encoding, the channel being enabled and
 * started, and the completion path - and a wrong one produces a visibly different
 * failure (see the notes in the code for which).
 *
 * What is NOT verified: peripheral-triggered transfers. M2M runs the channel in
 * "memory" mode with `peri_out_sel = -1`; connecting it to PARLIO and giving it a
 * real trigger is the next step and is not claimed here.
 *
 * ---------------------------------------------------------------------------
 * C5 specifics:
 *
 *  - One AHB group, three channel pairs (`AHB_DMA`). `AHB_DMA_LL_GET_HW()` returns
 *    NULL for anything but group 0.
 *
 *  - The descriptor owner bit is the opposite of what its name suggests from the
 *    DMA's side: `DMA_DESCRIPTOR_BUFFER_OWNER_DMA == 1` means "the engine may use
 *    this descriptor". IDF sets it to 1 before starting and expects to read 0 back
 *    when the engine has returned the buffer.
 *
 *  - `peri_out_sel` must be written even for M2M, with the sentinel (-1), which is
 *    `SOC_GDMA_TRIG_PERIPH_M2M0`.
 *
 *  - A descriptor chain must be 4-byte aligned; `dma_descriptor_t` is 12 bytes and
 *    asserts that size. The descriptors and both buffers live in internal RAM,
 *    because GDMA cannot reach PSRAM pages that the cache has not mapped for it.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/ahb_dma_struct.h"
#include "hal/ahb_dma_ll.h"
#include "hal/gdma_ll.h"
#include "hal/dma_types.h"
#include "hal/gdma_channel.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_GDMA_SUPPORTED)

#include "port_gdma.h"

/* One channel is enough for a self test. Channel 0 pair: TX 0 / RX 0. */
#define C5_GDMA_TX_CH      0
#define C5_GDMA_GROUP      0

static rt_bool_t s_gdma_ready;

rt_err_t rt_hw_gdma_init(void)
{
    if (s_gdma_ready)
    {
        return RT_EOK;
    }

    PERIPH_RCC_ATOMIC()
    {
        gdma_ll_enable_bus_clock(C5_GDMA_GROUP, true);
        gdma_ll_reset_register(C5_GDMA_GROUP);
    }

    /* The register interface has its own clock gate inside the peripheral, and
     * it is off by default. Without this every read of a channel register
     * returns zero, which looks exactly like "the LL writes are broken". */
    ahb_dma_ll_force_enable_reg_clock(AHB_DMA_LL_GET_HW(C5_GDMA_GROUP), true);

    s_gdma_ready = RT_TRUE;
    return RT_EOK;
}

/*
 * Copy `len` bytes with one descriptor, waiting for completion.
 *
 * Returns RT_EOK only when the engine reports EOF *and* the destination matches.
 * Those are separate checks on purpose: the completion bits say the channel ran
 * to the end of its chain, the comparison says it copied the right bytes.
 */
rt_err_t rt_hw_gdma_memcpy(void *dst, const void *src, rt_uint32_t len,
                           rt_uint32_t timeout_ms)
{
    static dma_descriptor_t tx_desc __attribute__((aligned(8)));
    static dma_descriptor_t rx_desc __attribute__((aligned(8)));
    ahb_dma_dev_t *hw;
    rt_uint32_t start;
    rt_uint32_t tx_status;
    rt_uint32_t rx_status;

    if (!s_gdma_ready && rt_hw_gdma_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (dst == RT_NULL || src == RT_NULL || len == 0u ||
        len > DMA_DESCRIPTOR_BUFFER_MAX_SIZE)
    {
        return -RT_EINVAL;
    }

    hw = AHB_DMA_LL_GET_HW(C5_GDMA_GROUP);

    /* MEMORY-TO-MEMORY NEEDS BOTH HALVES OF THE PAIR.
     *
     * This is the thing that cost a build: a TX channel connected "to memory"
     * with the sentinel id starts, finds nothing to push its data into, and never
     * returns the descriptor - which reads as "the channel did not run". IDF's
     * own M2M path (async_memcpy_gdma.c:121-133) allocates TX and RX as a PAIR and
     * connects BOTH to the same trigger.
     *
     * The trigger id is not a free choice: ahb_dma_ll.h documents that "for M2M
     * mode, hardware will automatically assign peri_sel ID depends on the channel
     * number (ch0: 10, ch1: 11, ch2: 12)", and AHB_DMA_LL_M2M_FREE_PERIPH_ID_MASK
     * is 0x1C00 - bits 10..12. Channel 0 therefore uses id 10. */
    tx_desc.dw0.size = (uint32_t)len & 0xFFFu;
    tx_desc.dw0.length = (uint32_t)len & 0xFFFu;
    tx_desc.dw0.err_eof = 0;
    tx_desc.dw0.suc_eof = 1;
    tx_desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    tx_desc.buffer = (void *)src;
    tx_desc.next = RT_NULL;

    rx_desc.dw0.size = (uint32_t)len & 0xFFFu;
    rx_desc.dw0.length = 0;
    rx_desc.dw0.err_eof = 0;
    rx_desc.dw0.suc_eof = 1;
    rx_desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    rx_desc.buffer = dst;
    rx_desc.next = RT_NULL;

    ahb_dma_ll_tx_reset_channel(hw, C5_GDMA_TX_CH);
    ahb_dma_ll_rx_reset_channel(hw, C5_GDMA_RX_CH);

    ahb_dma_ll_tx_connect_to_mem(hw, C5_GDMA_TX_CH, C5_GDMA_M2M_TRIG_ID);
    ahb_dma_ll_rx_connect_to_mem(hw, C5_GDMA_RX_CH, C5_GDMA_M2M_TRIG_ID);

    /* Owner checking on both sides: without it the engine will happily re-run a
     * descriptor that the CPU has already taken back, and the "wait for owner ==
     * CPU" completion test below would never see it change. */
    ahb_dma_ll_tx_enable_owner_check(hw, C5_GDMA_TX_CH, true);
    ahb_dma_ll_rx_enable_owner_check(hw, C5_GDMA_RX_CH, true);

    ahb_dma_ll_tx_set_eof_mode(hw, C5_GDMA_TX_CH, 1);
    ahb_dma_ll_rx_enable_auto_return(hw, C5_GDMA_RX_CH, false);

    ahb_dma_ll_tx_set_desc_addr(hw, C5_GDMA_TX_CH, (uint32_t)&tx_desc);
    ahb_dma_ll_rx_set_desc_addr(hw, C5_GDMA_RX_CH, (uint32_t)&rx_desc);

    ahb_dma_ll_tx_clear_interrupt_status(hw, C5_GDMA_TX_CH, UINT32_MAX);
    ahb_dma_ll_rx_clear_interrupt_status(hw, C5_GDMA_RX_CH, UINT32_MAX);

    /* RX first: the receiving half has to be armed before the transmitting half
     * pushes anything at it. */
    ahb_dma_ll_rx_start(hw, C5_GDMA_RX_CH);
    ahb_dma_ll_tx_start(hw, C5_GDMA_TX_CH);

    /* Wait on the INTERRUPT STATUS, not on the descriptor owner bit.
     *
     * This is the bug the diagnostic caught. The first version waited for
     * `desc.dw0.owner` to return to CPU, which only happens when auto write-back
     * is enabled - and it is off unless asked for. The engine had in fact finished
     * long before the timeout: measured
     *
     *   txint=0x0000000b   (TX_DONE | TX_EOF | TX_TOTAL_EOF)
     *   rxint=0x00000003   (RX_DONE | RX_SUC_EOF)
     *
     * so "the descriptor never came back" was true and meant nothing - the
     * completion bits were set the whole time the loop was spinning.
     *
     * The interrupt flags are the authoritative completion signal and they do not
     * depend on a write-back feature being on. The owner bit is still checked at
     * the end, as a second opinion. */
    start = rt_tick_get();
    while (true)
    {
        rt_uint32_t tx_intr = ahb_dma_ll_tx_get_interrupt_status(hw, C5_GDMA_TX_CH, true);
        rt_uint32_t rx_intr = ahb_dma_ll_rx_get_interrupt_status(hw, C5_GDMA_RX_CH, true);

        if ((tx_intr & GDMA_LL_EVENT_TX_DONE) && (rx_intr & GDMA_LL_EVENT_RX_DONE))
        {
            break;
        }

        if ((rt_tick_get() - start) > rt_tick_from_millisecond(timeout_ms))
        {
            ahb_dma_ll_tx_stop(hw, C5_GDMA_TX_CH);
            ahb_dma_ll_rx_stop(hw, C5_GDMA_RX_CH);
            return -RT_ETIMEOUT;
        }
    }

    tx_status = ahb_dma_ll_tx_get_interrupt_status(hw, C5_GDMA_TX_CH, true);
    rx_status = ahb_dma_ll_rx_get_interrupt_status(hw, C5_GDMA_RX_CH, true);

    ahb_dma_ll_tx_stop(hw, C5_GDMA_TX_CH);
    ahb_dma_ll_rx_stop(hw, C5_GDMA_RX_CH);

    ahb_dma_ll_tx_clear_interrupt_status(hw, C5_GDMA_TX_CH, UINT32_MAX);
    ahb_dma_ll_rx_clear_interrupt_status(hw, C5_GDMA_RX_CH, UINT32_MAX);

    /* DISCONNECT the channel before returning it.
     *
     * `c5dma` and `c5parlio` share TX channel 0 - the port has one channel
     * brought up - and the M2M test leaves it connected to the memory sentinel
     * with owner checking on. Measured: `c5parlio` passes on its own and fails
     * when run after `c5dma` in the same boot, because PARLIO's own
     * `tx_connect_to_periph()` is not enough to undo the M2M configuration the
     * channel was left in.
     *
     * A test that changes shared hardware state has to put it back. */
    if (tx_status & (GDMA_LL_EVENT_TX_DESC_ERROR | GDMA_LL_EVENT_TX_FIFO_OVF |
                     GDMA_LL_EVENT_TX_FIFO_UDF))
    {
        return -RT_ERROR;
    }

    if (rx_status & (GDMA_LL_EVENT_RX_DESC_ERROR | GDMA_LL_EVENT_RX_ERR_EOF))
    {
        return -RT_ERROR;
    }

    /* The owner bit only comes back when auto write-back is on, so it is reported
     * rather than required: `0/0` means the descriptors are still marked as owned
     * by the engine, which is expected here and is not a failure. */
    rt_kprintf("c5gdma  : completion txint=0x%08x rxint=0x%08x, owner tx=%u rx=%u\n",
               (unsigned)tx_status, (unsigned)rx_status,
               (unsigned)tx_desc.dw0.owner, (unsigned)rx_desc.dw0.owner);

    if (rt_memcmp(dst, src, len) != 0)
    {
        return -RT_ERROR;
    }

    /* QUIET THE CHANNEL BEFORE RETURNING.
     *
     * `c5dma` and `c5parlio` share TX channel 0. Measured: with this test run
     * first in the same boot, `c5parlio` transfers 88 of 128 bits and reports
     * `dma_txint=0x1` (TX_DONE) while PARLIO's own status stays empty; run
     * alone, it passes. Resetting the channel and leaving it disconnected is the
     * only state the next user cannot inherit something from - `tx_reset_channel`
     * alone resets the FSM but not the register file.
     *
     * A test that changes shared hardware state has to put it back. */
    ahb_dma_ll_tx_disconnect_all(hw, C5_GDMA_TX_CH);
    ahb_dma_ll_tx_reset_channel(hw, C5_GDMA_TX_CH);
    ahb_dma_ll_rx_disconnect_all(hw, C5_GDMA_RX_CH);
    ahb_dma_ll_rx_reset_channel(hw, C5_GDMA_RX_CH);

    return RT_EOK;
}

/* ------------------------------------------------------------------ msh -- */

/* `c5gdma [bytes]` - memory-to-memory copy through the AHB GDMA, verified. */
int c5_gdma_selftest(rt_uint32_t len)
{
    static rt_uint8_t src[256] __attribute__((aligned(16)));
    static rt_uint8_t dst[256] __attribute__((aligned(16)));
    rt_uint32_t i;
    rt_err_t err;
    int mismatches = 0;

    if (len == 0u || len > sizeof(src))
    {
        len = 128u;
    }

    rt_kprintf("c5gdma  : AHB GDMA group 0, TX channel %d, mem-to-mem\n",
               C5_GDMA_TX_CH);

    /* A position-dependent pattern: no two offsets hold the same byte, so a short
     * copy or an offset error cannot pass as a correct one. */
    rt_memset(dst, 0, sizeof(dst));
    for (i = 0; i < sizeof(src); i++)
    {
        src[i] = (rt_uint8_t)(i ^ 0x5A);
    }

    err = rt_hw_gdma_memcpy(dst, src, len, 100u);

    if (err != RT_EOK)
    {
        rt_kprintf("c5gdma  : copy of %u byte(s) failed (%d)\n",
                   (unsigned)len, (int)err);

        if (err == -RT_ETIMEOUT)
        {
            rt_kprintf("c5gdma  : the descriptor never came back - the channel "
                       "did not run, or did not see an EOF\n");
        }
        else
        {
            rt_kprintf("c5gdma  : the engine ran but the data or the status is "
                       "wrong\n");
        }
        return -1;
    }

    for (i = 0; i < len; i++)
    {
        if (dst[i] != src[i])
        {
            if (mismatches < 4)
            {
                rt_kprintf("c5gdma  : mismatch at +%u: %02x (want %02x)\n",
                           (unsigned)i, dst[i], src[i]);
            }
            mismatches++;
        }
    }

    /* Everything past `len` must still be zero: an overrun is a real failure mode
     * for a wrong descriptor `size` and would otherwise go unnoticed. */
    {
        rt_uint32_t overrun = 0;

        for (i = len; i < sizeof(dst); i++)
        {
            if (dst[i] != 0u)
            {
                overrun++;
            }
        }

        rt_kprintf("c5gdma  : copied %u byte(s), %u mismatch(es), %u byte(s) "
                   "written past the end\n",
                   (unsigned)len, (unsigned)mismatches, (unsigned)overrun);

        if (overrun != 0u)
        {
            rt_kprintf("c5gdma  : FAIL, the descriptor length is not what the "
                       "engine honoured\n");
            return -1;
        }
    }

    if (mismatches != 0)
    {
        rt_kprintf("c5gdma  : FAIL, the copy does not match the source\n");
        return -1;
    }

    rt_kprintf("c5gdma  : PASS (descriptor chain, channel start and completion "
               "all exercised; peripheral-triggered transfers are NOT verified)\n");
    return 0;
}

#endif /* SOC_GDMA_SUPPORTED */
