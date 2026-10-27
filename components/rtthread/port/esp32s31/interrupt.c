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
 */

#include <rthw.h>
#include <rtthread.h>

#include "soc/soc_caps.h"
#include "soc/interrupts.h"
#include "riscv/interrupt.h"
#include "esp_rom_sys.h"

#include "port_intr.h"

/* ETS INUM range. Slot arithmetic lives in the assembly, not here. */
#define RTT_INUM_NUM        SOC_CPU_INTR_NUM

struct rtt_isr_ctx
{
    rt_isr_handler_t fn;
    void            *param;
    int              vector;     /* ETS INUM, also what the ISR receives */
};

static struct rtt_isr_ctx s_ctx[RTT_INUM_NUM];

/* The only function with the signature IDF's dispatcher expects. */
void rtt_trampoline(void *arg)
{
    struct rtt_isr_ctx *c = (struct rtt_isr_ctx *)arg;

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

    s_ctx[vector] = (struct rtt_isr_ctx){ handler, param, vector };
    if (handler != RT_NULL)
    {
        intr_handler_set(vector, rtt_trampoline, &s_ctx[vector]);
    }
    else
    {
        intr_handler_set(vector, RT_NULL, RT_NULL);
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
        s_ctx[vector] = (struct rtt_isr_ctx){ RT_NULL, RT_NULL, vector };
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

void rt_hw_interrupt_mask(int vector)
{
    if (vector >= 0 && vector < RTT_INUM_NUM)
    {
        esprv_int_disable(1U << vector);
    }
}

void rt_hw_interrupt_umask(int vector)
{
    if (vector >= 0 && vector < RTT_INUM_NUM)
    {
        esprv_int_enable(1U << vector);
    }
}

/* Priority is the CLIC level (8-bit). Kept out of rthw.h on purpose: the
 * meaning of the value is SoC specific, so only SoC code should call it. */
void rt_hw_interrupt_set_priority(int vector, rt_uint32_t priority)
{
    if (vector >= 0 && vector < RTT_INUM_NUM)
    {
        esprv_int_set_priority(vector, (int)priority);
    }
}
