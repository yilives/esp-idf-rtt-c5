/*
 * ESP32-S31 interrupt bridge between RT-Thread and the ESP-IDF interrupt
 * matrix.
 *
 * Design notes (see the porting plan, section P2.1):
 *
 * 1. Numbering. RT-Thread's `vector` is the CPU interrupt line number, i.e.
 *    the ETS INUM used by esp_cpu_intr_set_handler() ("Interrupt number
 *    (from 0 to 31)"). It is NOT the CLIC vector-table slot: the hardware
 *    jumps to _mtvt_table[INUM + RV_EXTERNAL_INT_OFFSET] (offset 16), and
 *    _global_interrupt_handler() indexes back with `mcause - 16`. Passing a
 *    slot number here registers successfully but the handler never runs, so
 *    the range is asserted.
 *
 * 2. ABI. intr_handler_t is `void (*)(void *)` while rt_isr_handler_t is
 *    `void (*)(int, void *)`. esp_cpu_intr_set_handler() invokes the handler
 *    with a single argument, so a cast would hand the ISR `param` as its
 *    vector and garbage as its parameter. Every registration therefore goes
 *    through rtt_trampoline(), which owns the signature the IDF expects.
 *
 * 3. Sharing. UART0 and UART1 share ETS_UART0_INUM == ETS_UART1_INUM == 5.
 *    One slot per vector cannot represent that. Drivers that need sharing
 *    must call esp_intr_alloc() with an ETS_*_INTR_SOURCE and pass
 *    rtt_trampoline() plus their own context; this file's install API is for
 *    the single-owner case and for lines already routed by IDF/ROM.
 *
 * 4. Routing. Installing a handler is only step 2 of the four-step sequence
 *    in esp_rom_sys.h. Every INTERRUPT_CORE0_*_MAP register resets to 0, so
 *    the matrix starts with all sources on CPU line 0. rt_hw_intr_route()
 *    performs step 1; without it the handler is never reached and the only
 *    visible symptom is a peripheral that stays silent.
 *
 * 5. Level. On CLIC the threshold check is `int_ctl_byte > mintthresh_byte`,
 *    mintthresh is 0 (encoded 0x1F) and an external interrupt resets to level
 *    0 (also 0x1F). Installing a handler therefore has to assign a level as
 *    well, otherwise step 3 (unmask the line) is a no-op. See
 *    RTT_INTR_DEFAULT_LEVEL.
 */

#include <rthw.h>
#include <rtthread.h>

#include "soc/soc_caps.h"
#include "soc/interrupts.h"
#include "soc/clic_reg.h"
#include "riscv/interrupt.h"
#include "riscv/csr_clic.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"

#include "port_intr.h"

/* ETS INUM range. Slot arithmetic lives in the assembly, not here. */
#define RTT_INUM_NUM        SOC_CPU_INTR_NUM

/*
 * CLIC masks every interrupt whose level is <= mintthresh, and board.c programs
 * mintthresh to RVHAL_INTR_ENABLE_THRESH (0). An external interrupt also resets
 * to level 0, and CLIC_INT_THRESH encodes 0 as 0x1F, so the comparison
 * 0x1F > 0x1F fails: a line can be routed, enabled and unmasked and still never
 * be taken. IDF avoids this by always programming a level in esp_intr_alloc();
 * this bridge has to do the same. Level 1 is IDF's ESP_INTR_FLAG_LEVEL1, the
 * weakest level above the threshold.
 */
#define RTT_INTR_DEFAULT_LEVEL  1

/* Highest level the CLIC field can hold. NLBITS comes from the SoC's
 * csr_clic.h; 3 bits on this part, so 1..7. */
#define RTT_INTR_LEVEL_MAX      ((1u << NLBITS) - 1u)

struct rtt_isr_ctx
{
    rt_isr_handler_t fn;
    void            *param;
    int              vector;     /* ETS INUM, also what the ISR receives */
    rt_uint8_t       reported;   /* "fired with no handler" has been printed */
    rt_uint32_t      count;      /* entries, for the storm diagnosis */
};

static struct rtt_isr_ctx s_ctx[RTT_INUM_NUM];

/*
 * Per-line entry counters, readable from `s31intr`.
 *
 * Added because the PSRAM interrupt storm could not be attributed: the
 * watchdog dump names a PC inside a handler but not which line is being
 * re-entered, and reading the CLIC state afterwards shows nothing pending (the
 * line is level-triggered off a peripheral that has already been acknowledged).
 * A counter that only ever increments inside the trampoline answers "is this
 * line being entered in a loop" directly.
 *
 * RT-Thread callers must not read these from a thread without masking the line,
 * which is why the only consumer is the debug command.
 */
rt_uint32_t rt_hw_interrupt_entry_count(int vector)
{
    if (vector < 0 || vector >= RTT_INUM_NUM)
    {
        return 0;
    }

    return s_ctx[vector].count;
}

/* The only function with the signature IDF's dispatcher expects.
 *
 * IRAM_ATTR is load-bearing, not decoration.
 *
 * The trap entry (_interrupt_handler) and the dispatcher
 * (_global_interrupt_handler) are already in IRAM - the linker puts
 * .exception_vectors.text and the riscv component there. Everything below them
 * was in .flash.text, so the moment the cache is off the ISR cannot be fetched:
 * an SPI flash write/erase, or an MSPI/PSRAM operation, turns the next
 * peripheral interrupt into an instruction access fault or an interrupt-WDT
 * timeout with MEPC inside the handler.
 *
 * Measured on this board: with PSRAM enabled, `s31test irq` died with
 * "Interrupt wdt timeout on CPU0" and MEPC in s31_gpio_isr; with PSRAM disabled
 * the same test passed 200/200. That was not a PSRAM bug - it was this function
 * (and the handler behind it) living in flash. IDF marks every registered ISR
 * ESP_INTR_FLAG_IRAM for exactly this reason, and the hand-written drivers in
 * port/esp32s31 have to say it themselves. */
void IRAM_ATTR rtt_trampoline(void *arg)
{
    struct rtt_isr_ctx *c = (struct rtt_isr_ctx *)arg;

    /*
     * An unhandled level-triggered line is an interrupt storm, not a no-op.
     *
     * Every line this bridge installs is level-triggered (CLIC_INT_ATTR_TRIG
     * resets to level and nothing here programs it). So when a peripheral
     * asserts, the matrix routes it, the line is unmasked and the handler slot
     * is empty, the CPU takes the trap, this returns, mret re-enters
     * immediately - and no thread ever runs again. The only reporter is the
     * 300 ms interrupt watchdog, whose register dump shows the faulting PC as
     * this function.
     *
     * That is exactly what enabling PSRAM exposed: the MSPI/PSRAM sources raise
     * on CPU lines this port never installed anything on, and `s31test irq`
     * (which only cares about GPIO 4) was taken down by them.
     *
     * Masking on first sight turns a livelock into one line of output. Done
     * here rather than in the install path on purpose: this is the only place
     * that knows a line fired while nobody was listening.
     */
    if (c->fn == RT_NULL)
    {
        rt_hw_interrupt_mask(c->vector);

        if (!c->reported)
        {
            c->reported = 1;
            /* esp_rom_printf, not rt_kprintf: this runs on the trap path with the
             * flash cache possibly off, and rt_kprintf is in IROM. Measured:
             *
             *     rtt_trampoline  0x2f002e5a  IRAM
             *     rt_kprintf      0x400128ba  flash
             *     esp_rom_printf  0x2f800024  ROM
             *
             * The port has already seen a flash call from this path fault
             * ("MEPC 0x40013b1e resolved to rt_enter_critical in IROM"), and this
             * message is precisely the one that must survive the window where a
             * stray line is most likely. */
            esp_rom_printf("intr: vector %d fired with no handler, masked\n",
                           c->vector);
        }

        return;
    }

    /* s_ctx[].count is what `s31intr` prints to show a line is being re-entered
     * in a loop rather than serviced once. Volatile-ish by construction: the
     * increment happens with the line's own interrupt active, and the reader is
     * a debug command that accepts a torn value. */
    s_ctx[c->vector].count++;

    c->fn(c->vector, c->param);
}

/**
 * Install an ISR on a CPU interrupt line.
 *
 * @param vector ETS INUM (0..SOC_CPU_INTR_NUM-1), not the mtvt slot.
 *
 * The caller is responsible for having routed the peripheral source onto this
 * line (esp_intr_alloc() or esp_rom_route_intr_matrix()). Use esp_intr_alloc()
 * when the line may be shared, because this table holds a single owner.
 */
rt_isr_handler_t rt_hw_interrupt_install(int              vector,
                                         rt_isr_handler_t handler,
                                         void            *param,
                                         const char      *name)
{
    rt_isr_handler_t old;

    if (vector < 0 || vector >= RTT_INUM_NUM)
    {
        /* Catch the classic mistake: passing the mtvt slot (INUM + 16). */
        rt_kprintf("intr: bad vector %d for '%s' (want ETS INUM 0..%d)\n",
                   vector, name ? name : "?", RTT_INUM_NUM - 1);
        return RT_NULL;
    }

    old = s_ctx[vector].fn;

    /* Mask the line while the slot is rewritten.
     *
     * The assignment below is three word stores, not one, and the trampoline
     * reads all three. Re-installing on a line that is already enabled (a second
     * owner, or a driver that tears down and re-attaches) could take an
     * interrupt between the stores and call the new function with the old
     * parameter. A fresh line is at level 0 and therefore masked at the CLIC
     * threshold, so this only closes the re-install window - which is why
     * install-before-set_priority stays the right order and is unchanged. */
    {
        rt_uint32_t mask = 1U << vector;
        rt_bool_t   was_enabled = !rt_hw_interrupt_is_masked(vector);

        if (was_enabled)
        {
            rt_hw_interrupt_mask(vector);
        }

        s_ctx[vector] = (struct rtt_isr_ctx){ handler, param, vector, 0, 0 };

        if (handler != RT_NULL)
        {
            intr_handler_set(vector, rtt_trampoline, &s_ctx[vector]);

            /* Only when the line is still at its reset level, so a caller that
             * deliberately pre-set a level (rt_hw_interrupt_set_priority before
             * installing, which is what drivers wanting nesting do) keeps it. */
            if (esprv_int_get_priority(vector) == 0)
            {
                esprv_int_set_priority(vector, RTT_INTR_DEFAULT_LEVEL);
            }
        }
        else
        {
            intr_handler_set(vector, RT_NULL, RT_NULL);
        }

        if (was_enabled)
        {
            rt_hw_interrupt_umask(vector);
        }
    }

    return old;
}

void rt_hw_interrupt_uninstall(int vector, rt_isr_handler_t handler, void *param)
{
    if (vector < 0 || vector >= RTT_INUM_NUM)
    {
        return;
    }

    if (s_ctx[vector].fn == handler && s_ctx[vector].param == param)
    {
        /* Mask before dropping the handler.
         *
         * Every line this port installs is level-triggered: neither
         * rt_hw_interrupt_install() nor rt_hw_intr_route() programs
         * CLIC_INT_ATTR_TRIG, whose reset value selects level
         * (hal/interrupt_clic_ll.h: interrupt_clic_ll_get_type() reads it, and
         * IDF always sets the type explicitly in esp_intr_alloc). A level line
         * whose peripheral is still asserting, with no handler left, is an
         * immediate trap storm: _global_interrupt_handler finds NULL, returns,
         * mret re-enters the same trap, and no thread ever runs again.
         *
         * The old code left both the level and IE set, which is why the two
         * self tests that tear a line down (selftest.c st_nest/st_irq) had to
         * mask it by hand before calling this. */
        rt_hw_interrupt_mask(vector);

        s_ctx[vector] = (struct rtt_isr_ctx){ RT_NULL, RT_NULL, vector, 0, 0 };
        intr_handler_set(vector, RT_NULL, RT_NULL);
    }
}

int rt_hw_intr_route(int source, int inum)
{
    if (source < 0 || source >= ETS_MAX_INTR_SOURCE ||
        inum   < 0 || inum   >= RTT_INUM_NUM)
    {
        rt_kprintf("intr: bad route source=%d inum=%d (0..%d / 0..%d)\n",
                   source, inum, ETS_MAX_INTR_SOURCE - 1, RTT_INUM_NUM - 1);
        return -RT_EINVAL;
    }

    /* cpu_intr_num is the external interrupt number: the ROM adds
     * RV_EXTERNAL_INT_OFFSET itself, so the CLIC slot becomes inum + 16 and
     * _global_interrupt_handler() recovers it as mcause - 16. */
    esp_rom_route_intr_matrix(0, (uint32_t)source, (uint32_t)inum);

    return 0;
}

/*
 * ---------------------------------------------------------------------------
 * CLIC line enable, written directly.
 *
 * This is the second real defect the PSRAM storm uncovered, and it is worse
 * than the first: rt_hw_interrupt_mask()/umask() had NEVER worked.
 *
 * They called esprv_int_disable()/esprv_int_enable(), which look like the IDF
 * CLIC entry points but are not. The linker script maps them onto ROM:
 *
 *     PROVIDE (esprv_int_enable  = esprv_intc_int_enable)   0x2f800758  ROM
 *     PROVIDE (esprv_int_disable = esprv_intc_int_disable)  0x2f80075c  ROM
 *
 * Those are the *old INTC* implementations. components/riscv/interrupt_clic.c
 * defines the CLIC versions, but on this SoC the ROM PROVIDE wins and
 * interrupt_clic.c only contributes esprv_int_get_priority() and
 * intr_matrix_route() (confirmed in build/s31_rtt_test.map). The INTC version
 * does not touch the CLIC INTIE bit, so masking a line did nothing at all.
 *
 * Visible symptom: `s31intr -m 16` printed "masked line 16" and the next dump
 * still showed ie=1. Everything built on top of that - the unhandled-line sweep,
 * the trampoline's storm guard, the removal in rt_hw_interrupt_uninstall() - was
 * a no-op.
 *
 * Why the self tests still passed: the GPIO driver never relied on this path.
 * It has its own per-pin int_ena bits, and its loopback test drives the status
 * register directly. A green test suite said nothing about line masking.
 *
 * So the bit is written here, on the CLIC line index (INUM + 16, the same offset
 * interrupt_clic_ll_* uses), and nothing above it depends on ROM behaviour any
 * more.
 */
#define RTT_CLIC_LINE(inum)     (inum + RV_EXTERNAL_INT_OFFSET)
#define RTT_CLIC_CTRL(inum) \
    (*(volatile rt_uint32_t *)(CLIC_INT_CTRL_REG(RTT_CLIC_LINE(inum))))

static void IRAM_ATTR rtt_clic_set_ie(int inum, rt_bool_t on)
{
    volatile rt_uint32_t *reg = (volatile rt_uint32_t *)(
        CLIC_INT_CTRL_REG(RTT_CLIC_LINE(inum)));

    /* Read-modify-write: the same word holds the level and the trigger type, and
     * a blind store would reset both. IE is a plain R/W bit. */
    if (on)
    {
        *reg |= CLIC_INT_IE;
    }
    else
    {
        *reg &= ~CLIC_INT_IE;
    }
}

void rt_hw_interrupt_mask(int vector)
{
    if (vector >= 0 && vector < RTT_INUM_NUM)
    {
        rtt_clic_set_ie(vector, RT_FALSE);
    }
}

void rt_hw_interrupt_umask(int vector)
{
    if (vector >= 0 && vector < RTT_INUM_NUM)
    {
        rtt_clic_set_ie(vector, RT_TRUE);
    }
}

rt_bool_t rt_hw_interrupt_is_masked(int vector)
{
    if (vector < 0 || vector >= RTT_INUM_NUM)
    {
        return RT_TRUE;
    }

    return (RTT_CLIC_CTRL(vector) & CLIC_INT_IE) ? RT_FALSE : RT_TRUE;
}

/**
 * Mask every CPU line that is enabled but has no handler.
 *
 * This is the actual fix for the interrupt storm that made PSRAM unusable.
 *
 * The trampoline's `fn == RT_NULL` guard cannot help here, and finding that out
 * was the point of the whole investigation: _global_interrupt_handler()
 * (riscv/interrupt.c:59-69) looks up intr_get_item(mcause - 16) and calls the
 * handler *only when it is non-NULL*. A line that was never installed therefore
 * never reaches rtt_trampoline at all. It returns from the trap, mret re-enters
 * immediately, and no thread of ours ever runs again - only the 300 ms
 * interrupt watchdog reports anything, with MEPC inside _interrupt_handler.
 *
 * `s31intr` on a PSRAM-off build already shows four such lines (16, 18, 19, 21 -
 * LP_ADC, AHB_PDMA_IN_CH0, SYSTIMER_TARGET1, UART1): CLIC IE set, IDF handler
 * table empty. With PSRAM on the MSPI/CACHE sources add themselves to that set
 * and the storm starts as soon as anything touches flash.
 *
 * Masking them is safe by construction: no handler means no functionality is
 * lost, and any driver that legitimately wants the line will call
 * rt_hw_interrupt_install()/esp_intr_alloc() and re-enable it. Sweeping at the
 * end of board init rather than earlier matters, because the SECONDARY-stage
 * IDF inits allocate their lines after rt_hw_board_init() starts.
 *
 * Returns the number of lines masked.
 */
int rt_hw_intr_mask_unhandled(void)
{
    int inum;
    int masked = 0;

    for (inum = 0; inum < RTT_INUM_NUM; inum++)
    {
        /* Only lines the hardware is actually listening to. */
        if (rt_hw_interrupt_is_masked(inum))
        {
            continue;
        }

        if (intr_handler_get(inum) == RT_NULL && s_ctx[inum].fn == RT_NULL)
        {
            /*
             * Three lines are deliberately exempt: 24 (T1_WDT - the interrupt
             * watchdog), 25 (CACHEERR) and 27 (ASSIST_DEBUG).
             *
             * They have no entry in the IDF handler table by design: IDF hands
             * them straight to an assembly panic path rather than a C handler
             * ("we do this manually as the timeout interrupt should call an
             * assembly panic handler", int_wdt.c:181-184). Masking the
             * interrupt watchdog would silently remove the one mechanism that
             * reports a hung image - the opposite of what this sweep is for.
             */
            if (inum == ETS_T1_WDT_INUM ||
                inum == ETS_CACHEERR_INUM ||
                inum == ETS_ASSIST_DEBUG_INUM)
            {
                continue;
            }

            rt_hw_interrupt_mask(inum);
            masked++;
        }
    }

    return masked;
}

/* Priority is the CLIC level (8-bit field, but only NLBITS bits are meaningful).
 * Kept out of rthw.h on purpose: the meaning of the value is SoC specific, so
 * only SoC code should call it.
 *
 * The range check is load-bearing. The hardware keeps NLBITS bits of the level
 * (csr_clic.h: NLBITS_MASK = (1 << NLBITS) - 1, i.e. 0..7 on this part), and
 * interrupt_clic_ll_get_priority() shifts the field down by (8 - NLBITS). A
 * value of 8 therefore lands in the bits that get shifted away, reads back as 0,
 * and a level of 0 is *masked* at the threshold board.c programs - so the
 * interrupt silently stops arriving with no diagnostic anywhere. */
void rt_hw_interrupt_set_priority(int vector, rt_uint32_t priority)
{
    if (vector < 0 || vector >= RTT_INUM_NUM)
    {
        return;
    }

    if (priority == 0 || priority > RTT_INTR_LEVEL_MAX)
    {
        rt_kprintf("intr: level %u out of range 1..%u for vector %d\n",
                   (unsigned)priority, (unsigned)RTT_INTR_LEVEL_MAX, vector);
        return;
    }

    /* A line that is already enabled is being re-levelled: mask it for the
     * duration of the write so the arbitration unit cannot sample a half-written
     * level. */
    {
        rt_uint32_t mask = 1U << vector;
        rt_bool_t   was_enabled;

        was_enabled = !rt_hw_interrupt_is_masked(vector);
        if (was_enabled)
        {
            rt_hw_interrupt_mask(vector);
        }

        esprv_int_set_priority(vector, (int)priority);

        if (was_enabled)
        {
            rt_hw_interrupt_umask(vector);
        }
    }
}
