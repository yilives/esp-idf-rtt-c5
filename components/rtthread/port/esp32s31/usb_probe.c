/*
 * USB OTG capability probe.
 *
 * The question "how many endpoints does this chip have, and how many USB devices can
 * it present" is answered by the core itself: a Synopsys DWC OTG controller publishes
 * its own configuration in GHWCFG1..GHWCFG4, which are read-only registers synthesised
 * into the RTL. Reading them beats quoting a datasheet, because the datasheet is for the
 * family and these registers are per-instance.
 *
 * What is decoded here:
 *
 *   GHWCFG2.NUMDEVEPS   number of device endpoints, EXCLUDING endpoint 0. The core
 *                       reports N here but the usable range is 0..N, so the total
 *                       endpoint count is N+1 - that is what CherryUSB does too
 *                       (`num_dev_ep + 1` all over usb_dc_dwc2.c).
 *   GHWCFG4.INEPS       of those, how many can be IN.
 *   GHWCFG4.NUMDEVPERIOEPS  of those IN endpoints, how many can be periodic (isochronous /
 *                       interrupt). The rest are bulk-only.
 *   GHWCFG1.EPDIR       one bit per endpoint: 0 = bidirectional (both directions share the
 *                       endpoint number), 1 = unidirectional. This decides whether a
 *                       function can use OUT n and IN n at the same time.
 *   GHWCFG3.DFIFODEPTH  total FIFO depth in 32-bit words. Endpoint memory is a shared
 *                       resource: more endpoints does not mean more buffering.
 *   GHWCFG2.ARCH        DMA architecture. 2 = internal DMA, which is what CherryUSB's
 *                       dwc2 port asserts on.
 *   GHWCFG2.HSPHYTYPE / FSPHYTYPE   which PHY the core was synthesised with. S31 has a
 *                       UTMI PHY (`SOC_USB_FSLS_PHY_NUM == 0`, `SOC_USB_UTMI_PHY_NUM ==
 *                       1`), i.e. high speed only, no full-speed fallback PHY.
 *
 * Everything is read after enabling the peripheral clock, and the clock is left on: the
 * probe is a read-only inspection and turning the clock back off would make a second run
 * of `s31usb` report a bus-hung core instead of the same numbers.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "soc/reg_base.h"
#include "soc/usb_dwc_struct.h"
#include "soc/hp_sys_clkrst_reg.h"
#include "soc/cnnt_sys_reg.h"
#include "soc/hp_alive_sys_reg.h"
#include "soc/interrupts.h"

#define S31_USB_OTGHS ((usb_dwc_dev_t *)DR_REG_USB_OTGHS_BASE)

/*
 * cnnt_sys_reg.h defines CNNT_SYS_USB_OTG20_CTRL_REG in terms of DR_REG_CNNT_BASE, but
 * reg_base.h only names the peripheral DR_REG_CNNT_SYS_REG_BASE (0x20359000). Substituting
 * it here is a one-line fix inside the port and keeps the vendor header untouched.
 */
#undef CNNT_SYS_USB_OTG20_CTRL_REG
#define CNNT_SYS_USB_OTG20_CTRL_REG (DR_REG_CNNT_SYS_REG_BASE + 0x30)

/*
 * Bring the controller out of reset with a clock, so its registers answer.
 *
 * Measured: setting only HP_SYS_CLKRST.{APB,SYS}_CLK_EN was not enough - every GHWCFG
 * register read back as zero, including GSNPSID, which is the Synopsys ID and cannot be
 * zero on a working core. The full chain has four parts, and the UTMI HAL does all four
 * (esp_hal_usb/esp32s31/include/hal/usb_utmi_ll.h):
 *
 *   1. HP_SYS_CLKRST.usb_otghs_ctrl0: apb_clk_en + sys_clk_en
 *   2. CNNT_SYS.usb_otg20_ctrl:       utmifs_clk_en + phyref_clk_en
 *   3. CNNT_SYS.usb_otg20_ctrl:       de-assert phy_rst, then ahb_rst, then apb_rst
 *                                     (PHY first - it has to be stable before the
 *                                      controller starts)
 *   4. HP_ALIVE_SYS.usb_otghs_ctrl:   clear phy_suspendm_force_en so the DWC can drive
 *                                     PHY suspend/resume itself
 */
static void s31_usb_clk_on(void)
{
    REG_SET_BIT(HP_SYS_CLKRST_USB_OTGHS_CTRL0_REG,
                HP_SYS_CLKRST_REG_USB_OTGHS_APB_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_USB_OTGHS_CTRL0_REG,
                HP_SYS_CLKRST_REG_USB_OTGHS_SYS_CLK_EN);

    REG_SET_BIT(CNNT_SYS_USB_OTG20_CTRL_REG, CNNT_SYS_USB_OTG20_UTMIFS_CLK_EN);
    REG_SET_BIT(CNNT_SYS_USB_OTG20_CTRL_REG, CNNT_SYS_USB_OTG20_PHYREF_CLK_EN);

    /* PHY reset before the bus resets, per the HAL's ordering note. */
    REG_CLR_BIT(CNNT_SYS_USB_OTG20_CTRL_REG, CNNT_SYS_USB_OTG20_PHY_RST_EN);
    REG_CLR_BIT(CNNT_SYS_USB_OTG20_CTRL_REG, CNNT_SYS_USB_OTG20_AHB_RST_EN);
    REG_CLR_BIT(CNNT_SYS_USB_OTG20_CTRL_REG, CNNT_SYS_USB_OTG20_APB_RST_EN);

    REG_CLR_BIT(HP_ALIVE_SYS_USB_OTGHS_CTRL_REG,
                HP_ALIVE_SYS_REG_USB_OTGHS_PHY_SUSPENDM_FORCE_EN);
}

static const char *s31_usb_arch_name(unsigned arch)
{
    switch (arch)
    {
    case 0: return "slave-only (no DMA)";
    case 1: return "external DMA";
    case 2: return "internal DMA";
    default: return "reserved";
    }
}

static const char *s31_usb_hs_phy_name(unsigned v)
{
    switch (v)
    {
    case 0: return "none";
    case 1: return "UTMI+";
    case 2: return "ULPI";
    case 3: return "UTMI+ and ULPI";
    default: return "reserved";
    }
}

static const char *s31_usb_fs_phy_name(unsigned v)
{
    switch (v)
    {
    case 0: return "none (no full-speed fallback)";
    case 1: return "dedicated full-speed";
    case 2: return "shared with UTMI+";
    case 3: return "shared with ULPI";
    default: return "reserved";
    }
}

int cmd_usb(int argc, char **argv)
{
    usb_dwc_dev_t *hw = S31_USB_OTGHS;
    rt_uint32_t cfg1, cfg2, cfg3, cfg4;
    unsigned n_ep, n_in, n_perio_in, arch, hs_phy, fs_phy;
    rt_uint32_t fifo_words;
    int i;

    s31_usb_clk_on();

    cfg1 = hw->ghwcfg1_reg.val;
    cfg2 = hw->ghwcfg2_reg.val;
    cfg3 = hw->ghwcfg3_reg.val;
    cfg4 = hw->ghwcfg4_reg.val;

    /* NUMDEVEPS excludes EP0; the usable endpoint numbers are 0..NUMDEVEPS. */
    n_ep       = (cfg2 >> 10) & 0xFu;
    n_in       = (cfg4 >> 26) & 0xFu;
    n_perio_in = (cfg4 >> 0)  & 0xFu;
    arch       = (cfg2 >> 3)  & 0x3u;
    hs_phy     = (cfg2 >> 6)  & 0x3u;
    fs_phy     = (cfg2 >> 8)  & 0x3u;
    fifo_words = (cfg3 >> 16) & 0xFFFFu;

    rt_kprintf("s31usb: Synopsys ID 0x%08x (DWC OTG rev %u.%02x)\n",
               (unsigned)hw->gsnpsid_reg.val,
               (unsigned)((hw->gsnpsid_reg.val >> 24) & 0xFFu),
               (unsigned)((hw->gsnpsid_reg.val >> 16) & 0xFFu));

    rt_kprintf("s31usb: GHWCFG1=%08x GHWCFG2=%08x GHWCFG3=%08x GHWCFG4=%08x\n",
               (unsigned)cfg1, (unsigned)cfg2, (unsigned)cfg3, (unsigned)cfg4);

    rt_kprintf("s31usb: device endpoints (EP0 included) : %u  (%u IN, %u periodic IN)\n",
               n_ep + 1u, n_in, n_perio_in);

    /*
     * Bidirectional or unidirectional, endpoint by endpoint.
     *
     * GHWCFG1.EPDIR bit n is 0 when endpoint n is bidirectional and 1 when it is a
     * single direction. Most functions need bidirectional EP0 plus one bulk IN and one
     * bulk OUT; a chip where every endpoint is bidirectional can present a CDC-ACM
     * (EP0 + IN + OUT) in two endpoints, while a unidirectional core needs three.
     */
    rt_kprintf("s31usb: endpoint directions             : ");
    for (i = 0; i <= (int)n_ep && i < 32; i++)
    {
        rt_kprintf("%u%s", i,
                   ((cfg1 >> i) & 1u) ? (char *)" uni" : (char *)" bi");
        if (i != (int)n_ep)
        {
            rt_kprintf(", ");
        }
    }
    rt_kprintf("\n");

    rt_kprintf("s31usb: DMA architecture               : %u (%s)\n",
               arch, s31_usb_arch_name(arch));
    rt_kprintf("s31usb: HS PHY %s, FS PHY %s\n",
               s31_usb_hs_phy_name(hs_phy), s31_usb_fs_phy_name(fs_phy));
    rt_kprintf("s31usb: total FIFO %u words = %u bytes on this core\n",
               (unsigned)fifo_words, (unsigned)(fifo_words * 4u));

    rt_kprintf("s31usb: interrupt sources              : OTGHS=%d, multi-proc=%d, "
               "serial-jtag=%d\n",
               ETS_USB_OTGHS_INTR_SOURCE,
               ETS_USB_OTGHS_ENDP_MULTI_PROC_INTR_SOURCE,
               ETS_USB_SERIAL_JTAG_INTR_SOURCE);

    /*
     * What the practical ceiling is for "how many USB devices can this be".
     *
     * One physical port, so one device as far as a host is concerned - but that device
     * can present many interfaces, and the endpoint count is what bounds it. State the
     * bound rather than a number of "devices", because a composite function is still one
     * device with many interfaces.
     */
    rt_kprintf("s31usb: one physical port; a composite device can use at most %u "
               "endpoint(s)\n", n_ep + 1u);

    /*
     * Isochronous capability, which is the one limit that changes what is buildable.
     *
     * GHWCFG4.NUMDEVPERIOEPS is the number of periodic IN endpoints the core was
     * synthesised with. Measured 0 on this instance, so there is no isochronous endpoint
     * at all - which rules out USB audio (UAC) and USB video (UVC) streaming, and leaves
     * bulk/interrupt/interrupt-control functions unaffected (CDC, MSC, HID, MIDI, vendor).
     */
    if (n_perio_in == 0)
    {
        rt_kprintf("s31usb: isochronous endpoints           : 0 (no UAC / UVC streaming)\n");
    }
    else
    {
        rt_kprintf("s31usb: isochronous endpoints           : %u\n", n_perio_in);
    }

    if (argc > 1 && rt_strcmp(argv[1], "-d") == 0)
    {
        rt_kprintf("s31usb: DIEPCTL0=%08x DOEPCTL0=%08x DCFG=%08x DSTS=%08x\n",
                   (unsigned)hw->diepctl0_reg.val,
                   (unsigned)hw->doepctl0_reg.val,
                   (unsigned)hw->dcfg_reg.val,
                   (unsigned)hw->dsts_reg.val);
    }

    (void)i;
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_usb, s31usb, show the USB OTG endpoint and PHY capabilities);
