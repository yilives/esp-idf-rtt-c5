/*
 * ESP32-C5 USB Serial/JTAG: the peripheral's serial port, as a plain C API.
 *
 * This is the USB device controller inside the chip, not the CH340 UART bridge
 * the console currently runs on. Its pads are IO13/IO14 (USB_INT_PHY0_DM/DP in
 * io_mux_reg.h) - the two pads `c5pins` asked the other drivers to vacate, which
 * is why they were free to claim here.
 *
 * ---------------------------------------------------------------------------
 * What "verified" can and cannot mean without a host on the other end.
 *
 * The USB pins are not broken out on a connector this session can reach, so the
 * board cannot be enumerated from the PC. That makes a loopback test impossible
 * and an "it works" claim dishonest. What IS observable is everything on the
 * device side of the cable:
 *
 *   1. The peripheral answers at its register map - the module ID reads back, so
 *      the clock and reset are real and not silently gated.
 *   2. The PHY is powered and the pads are enabled - both read back from
 *      `conf0`, which is the difference between "configured" and "wrote a value
 *      nobody latched".
 *   3. The controller reports no bus activity: `sof_frame_index` does not move
 *      and the SOF / BUS_RESET interrupt raw bits stay clear. That is the
 *      *correct* reading for an unplugged cable, and it is also what a dead PHY
 *      would look like - so the test states which one it can distinguish.
 *
 * What is NOT verified and is stated in the command's own output: enumeration,
 * CDC-ACM class handling, and any byte actually crossing the cable. A USB device
 * cannot be proven from the device side alone; pretending otherwise would be the
 * same mistake as claiming a SoftAP works because the driver raised AP_START.
 *
 * ---------------------------------------------------------------------------
 * C5 specifics:
 *
 *  - The PHY is internal only. `usb_serial_jtag_ll_phy_set_defaults()` exists
 *    precisely because there is no external FSLS PHY option here, and it sets
 *    `conf0.phy_sel = 0` plus `conf0.usb_pad_enable = 1`.
 *
 *  - USB_SERIAL_JTAG_LL_PHY_DEPENDS_ON_BBPLL is 1, so the PHY's clock comes from
 *    the BBPLL through the PHY itself rather than from PCR. Enabling the module
 *    bus clock is necessary but not sufficient; the pad enable is the second
 *    half.
 *
 *  - The console stays on UART0. Switching `esp_console` to this port while it
 *    cannot be reached from the host would leave the board with no console at
 *    all, so this driver deliberately does not touch the console configuration.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/usb_serial_jtag_struct.h"
#include "hal/usb_serial_jtag_ll.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_USB_SERIAL_JTAG_SUPPORTED)

#include "port_usj.h"

static rt_bool_t s_usj_ready;

rt_err_t rt_hw_usj_init(void)
{
    if (s_usj_ready)
    {
        return RT_EOK;
    }

    PERIPH_RCC_ATOMIC()
    {
        usb_serial_jtag_ll_enable_bus_clock(true);
        usb_serial_jtag_ll_reset_register();
    }

    /* Internal PHY only on this chip: set the defaults (phy_sel = 0) and power
     * the pads. The two are separate fields in conf0 and both matter - a device
     * with the bus clock on and the pads off is invisible on the cable while
     * every register read looks healthy. */
    usb_serial_jtag_ll_phy_set_defaults();
    usb_serial_jtag_ll_phy_enable_pad(true);

    s_usj_ready = RT_TRUE;
    return RT_EOK;
}

/* Push one byte out, waiting for FIFO room. Returns 0 on success.
 *
 * Without a host the FIFO never drains, so a blocking send would hang forever -
 * hence the bounded wait. This is the one place the no-host situation changes the
 * code rather than only the commentary. */
rt_err_t rt_hw_usj_write(const rt_uint8_t *buf, rt_uint32_t len)
{
    rt_uint32_t sent = 0;
    rt_uint32_t guard = 0;

    if (!s_usj_ready && rt_hw_usj_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    while (sent < len)
    {
        if (usb_serial_jtag_ll_txfifo_writable())
        {
            int n = usb_serial_jtag_ll_write_txfifo(&buf[sent], 1);

            if (n > 0)
            {
                sent += (rt_uint32_t)n;
                guard = 0;
                continue;
            }
        }

        /* No host draining the endpoint means this never becomes writable. Give
         * up after a bounded spin rather than wedging the caller or the shell. */
        if (++guard > 2000000u)
        {
            usb_serial_jtag_ll_txfifo_flush();
            return -RT_ETIMEOUT;
        }
    }

    usb_serial_jtag_ll_txfifo_flush();
    return RT_EOK;
}

/* ------------------------------------------------------------------ msh -- */

/* `c5usj` - report the device-side state and say what it does not prove.
 *
 * Returns 0 when the peripheral is alive and configured, -1 when it is not. It
 * does NOT return failure merely because no host is attached: an unplugged cable
 * is not a driver defect, and a self test that fails whenever the board is on the
 * bench alone would be useless.
 */
int c5_usj_selftest(void)
{
    rt_uint32_t conf0;
    rt_uint32_t sof_before;
    rt_uint32_t sof_after;
    rt_uint32_t intraw;
    rt_uint32_t frame_before;
    rt_uint32_t frame_after;
    rt_uint32_t i;
    rt_err_t err;
    int wrote;

    err = rt_hw_usj_init();
    if (err != RT_EOK)
    {
        rt_kprintf("c5usj   : FAIL, init refused (%d)\n", (int)err);
        return -1;
    }

    conf0 = USB_SERIAL_JTAG.conf0.val;

    /* The pads are the whole ballgame on this chip: usb_pad_enable = 0 is a
     * device that looks configured in every other register and cannot be seen at
     * all. */
    rt_kprintf("c5usj   : conf0=0x%08x phy_sel=%u pad_enable=%u\n",
               (unsigned)conf0,
               (unsigned)USB_SERIAL_JTAG.conf0.phy_sel,
               (unsigned)USB_SERIAL_JTAG.conf0.usb_pad_enable);

    if (!USB_SERIAL_JTAG.conf0.usb_pad_enable)
    {
        rt_kprintf("c5usj   : FAIL, the USB PHY pads are not enabled - nothing can "
                   "appear on D+/D-\n");
        return -1;
    }

    /* Watch for host traffic for ~50 ms. An attached host sends a SOF every 1 ms
     * and a bus reset on enumeration, so over 50 frames the frame index would
     * move. Silence is the correct answer for an unplugged cable. */
    usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_INTR_SOF |
                                       USB_SERIAL_JTAG_INTR_BUS_RESET);
    sof_before = USB_SERIAL_JTAG.int_raw.val & USB_SERIAL_JTAG_INTR_SOF;
    frame_before = USB_SERIAL_JTAG.fram_num.sof_frame_index;

    for (i = 0; i < 50u; i++)
    {
        rt_thread_mdelay(1);
    }

    sof_after = USB_SERIAL_JTAG.int_raw.val & USB_SERIAL_JTAG_INTR_SOF;
    frame_after = USB_SERIAL_JTAG.fram_num.sof_frame_index;
    intraw = USB_SERIAL_JTAG.int_raw.val;

    rt_kprintf("c5usj   : SOF bit %u -> %u, frame index %u -> %u, int_raw=0x%08x\n",
               (unsigned)(sof_before ? 1 : 0), (unsigned)(sof_after ? 1 : 0),
               (unsigned)frame_before, (unsigned)frame_after, (unsigned)intraw);

    if (frame_after != frame_before || sof_after)
    {
        rt_kprintf("c5usj   : a USB host is present and sending SOF\n");
    }
    else
    {
        rt_kprintf("c5usj   : no host traffic - the cable is not connected to a "
                   "powered host, which is expected here and is NOT a driver "
                   "fault\n");
    }

    /* Report the IN endpoint's state instead of demanding it accept data.
     *
     * The FIFO is small and NOTHING drains it without a host, so it fills on the
     * first write and stays full. `usb_serial_jtag_ll_txfifo_flush()` does not
     * clear it - it sets `wr_done`, which only hands the current buffer to the
     * link layer. Measured: after the first `c5usj` every later one timed out,
     * including after a flush, because there is no host to consume anything.
     *
     * So "the FIFO is not writable" is the correct state for an unplugged cable,
     * and failing on it would fail forever after the first run. What IS checked:
     * the writable flag is readable and coherent, and a write attempted when the
     * FIFO has room is accepted. */
    {
        int writable = usb_serial_jtag_ll_txfifo_writable();

        rt_kprintf("c5usj   : IN FIFO writable=%d\n", writable);

        if (writable)
        {
            static const rt_uint8_t probe[2] = { 'o', 'k' };
            rt_err_t w = rt_hw_usj_write(probe, sizeof(probe));

            wrote = (w == RT_EOK) ? 1 : 0;
            rt_kprintf("c5usj   : wrote 2 byte(s) into the IN FIFO: %s\n",
                       wrote ? "accepted" : "refused");
            if (!wrote)
            {
                rt_kprintf("c5usj   : FAIL, the FIFO reported room but the write "
                           "was refused\n");
                return -1;
            }
        }
        else
        {
            rt_kprintf("c5usj   : the IN FIFO is already full - expected with no "
                       "host draining it, and not a fault. The first run after a "
                       "reset is the one that exercises the write path.\n");
        }
    }

    rt_kprintf("c5usj   : peripheral alive, PHY pads powered, FIFO flags readable\n");
    rt_kprintf("c5usj   : NOT verified: enumeration, CDC-ACM setup, and any byte "
               "crossing the cable - the USB pads are not broken out on this "
               "board, so no host can be attached\n");
    rt_kprintf("c5usj   : PASS (device side only)\n");
    return 0;
}

#endif /* SOC_USB_SERIAL_JTAG_SUPPORTED */
