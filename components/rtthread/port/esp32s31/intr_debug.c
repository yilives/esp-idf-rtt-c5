/*
 * `s31intr` - dump the CLIC line state.
 *
 * Written because the PSRAM interrupt storm could not be localised by reading
 * code: the faulting register dump said only "MCAUSE=0x18" and the PC, never
 * which line was asserting.
 *
 * What the dump has to distinguish, and why the obvious guard was not enough:
 *
 *   _global_interrupt_handler() (riscv/interrupt.c:59) looks up
 *   intr_get_item(mcause - 16) and calls the handler *only if it is non-NULL*.
 *   So a line that was never installed does not reach rtt_trampoline at all -
 *   my "if (c->fn == RT_NULL) mask it" guard is dead code for that case. A
 *   level-triggered line whose CLIC IE bit is set and whose handler slot is
 *   empty therefore returns from the trap and re-enters immediately, forever,
 *   with no code of ours ever running. The only witness is the interrupt
 *   watchdog.
 *
 * So the thing that actually has to be checked is the pair
 *   (CLIC IE bit set) AND (no handler in the IDF table)
 * and this command prints exactly that pair, plus the level and trigger type
 * that decide whether a stray source turns into a storm or a single edge.
 */

#include <rthw.h>
#include <rtthread.h>

#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/interrupts.h"
#include "soc/clic_reg.h"
#include "soc/reg_base.h"
#include "riscv/interrupt.h"
#include "riscv/csr_clic.h"

#include "port_intr.h"

/* CLIC per-line control word. The LL helpers only expose priority/type, and the
 * IE bit is what matters here. */
/* The offset is load-bearing and was missing in the first version: CLIC line
 * N + 16 is external interrupt N (csr_clic.h, RV_EXTERNAL_INT_OFFSET), so
 * indexing the word by INUM directly dumps the sixteen *internal* lines and
 * labels them with external numbers. That misreading sent me looking for
 * handlers on lines 0/2/3/5 and made the real ones (16/18/19/21 behind
 * LP_ADC / AHB_PDMA_IN_CH0 / SYSTIMER_TARGET1 / UART1) look handled. */
#define S31_CLIC_LINE_REG(n)    (*(volatile rt_uint32_t *)(CLIC_INT_CTRL_REG((n) + RV_EXTERNAL_INT_OFFSET)))
#define S31_CLIC_IE             CLIC_INT_IE      /* bit 8 */
#define S31_CLIC_IP             CLIC_INT_IP      /* bit 0  */

static rt_uint32_t s31_clic_priority(int inum)
{
    /* Two shifts, and both are load-bearing.
     *
     * CLIC_INT_CTL is a byte at [31:24], and the level lives in its top NLBITS
     * bits with the low (8 - NLBITS) bits tied high - that is what
     * NLBITS_TO_BYTE() in csr_clic.h does. interrupt_clic_ll_get_priority()
     * therefore shifts the byte down by (8 - NLBITS) after extracting it.
     *
     * Printing the raw byte instead shows 0x1F for level 0 (masked) and 0x3F
     * for level 1 (enabled), which reads like "31" and "63" and looks like
     * nonsense. The first version of this command did exactly that. */
    return ((S31_CLIC_LINE_REG(inum) & CLIC_INT_CTL_M) >> CLIC_INT_CTL_S)
           >> (8 - NLBITS);
}

static const char *s31_line_name(int inum)
{
    /* esp_isr_names is indexed by *source*, not by line - a line can be shared.
     * For the diagnostic, report the first source currently routed onto it.
     *
     * The matrix register for source N is at CORE0_BASE + 4*N and holds the
     * destination line in bits [5:0] (RV_INT_MASK = 63). Reading it is the only
     * way to answer "who is on this line" without the IDF allocator's metadata,
     * which this port does not populate for the lines IDF never allocated. */
    static char buf[40];
    int i;

    for (i = 0; i < ETS_MAX_INTR_SOURCE; i++)
    {
        rt_uint32_t m = *(volatile rt_uint32_t *)(DR_REG_INTERRUPT_CORE0_BASE + 4u * (rt_uint32_t)i);

        /* The matrix stores the CLIC line (inum + 16), not the ETS INUM: the ROM
         * helper adds RV_EXTERNAL_INT_OFFSET itself. Comparing against inum
         * alone matches nothing and every line prints "-". */
        if ((m & 0x3Fu) == (rt_uint32_t)(inum + RV_EXTERNAL_INT_OFFSET))
        {
            rt_snprintf(buf, sizeof(buf), "%s",
                        esp_isr_names[i] ? esp_isr_names[i] : "?");
            return buf;
        }
    }

    return "-";
}

static int cmd_intr(int argc, char **argv)
{
    int inum;
    int enabled_empty = 0;
    rt_bool_t verbose = (argc >= 2 && rt_strcmp(argv[1], "-a") == 0);

    rt_kprintf("inum ie ip lvl trig handler     cnt name\n");
    rt_kprintf("---- -- -- --- ---- ------- ------- ----\n");

    for (inum = 0; inum < SOC_CPU_INTR_NUM; inum++)
    {
        rt_uint32_t reg = S31_CLIC_LINE_REG(inum);
        rt_uint32_t ie = (reg & S31_CLIC_IE) ? 1u : 0u;
        rt_uint32_t ip = (reg & S31_CLIC_IP) ? 1u : 0u;
        rt_uint32_t lvl = s31_clic_priority(inum);
        rt_uint32_t trig = (reg >> CLIC_INT_ATTR_TRIG_S) & 3u;
        intr_handler_t h = intr_handler_get(inum);

        /* Only the interesting lines by default: enabled, or pending, or with a
         * handler. 32 rows of all-zero is noise. */
        if (!verbose && ie == 0 && ip == 0 && h == RT_NULL)
        {
            continue;
        }

        rt_kprintf("%4d %2u %2u %3u %4s %-7s %7u %s\n",
                   inum, (unsigned)ie, (unsigned)ip, (unsigned)lvl,
                   (trig == 0) ? "lvl" : "edge",
                   (h != RT_NULL) ? "yes" : "NO",
                   (unsigned)rt_hw_interrupt_entry_count(inum),
                   s31_line_name(inum));

        if (ie && h == RT_NULL)
        {
            enabled_empty++;
        }
    }

    if (enabled_empty)
    {
        rt_kprintf("\n%d line(s) are ENABLED with NO handler.\n", enabled_empty);
        rt_kprintf("A level-triggered one of those is an interrupt storm: the trap\n");
        rt_kprintf("returns, mret re-enters, and no thread runs again. Mask with\n");
        rt_kprintf("`s31intr -m <inum>` or find who enabled it.\n");
    }
    else
    {
        rt_kprintf("\nno enabled-but-unhandled lines\n");
    }

    if (argc >= 3 && rt_strcmp(argv[1], "-m") == 0)
    {
        int v = atoi(argv[2]);

        if (v >= 0 && v < SOC_CPU_INTR_NUM)
        {
            rt_hw_interrupt_mask(v);
            rt_kprintf("masked line %d\n", v);
        }
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_intr, s31intr, dump CLIC line state; -a for all and -m N to mask);

/* Same --gc-sections anchor as the other段注册 files: nothing references this
 * object's symbols, so without a global that somebody calls it never leaves
 * librtthread.a and the command silently does not exist. */
void s31_intr_debug_link_anchor(void)
{
}
