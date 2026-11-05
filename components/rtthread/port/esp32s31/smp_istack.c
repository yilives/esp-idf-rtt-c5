/*
 * Per-CPU interrupt stacks.
 *
 * The problem
 * -----------
 * The linker allocates ONE interrupt stack (esp_system/ld/esp32s31/sections.ld.in
 * .stack, CONFIG_IDF_RTTHREAD_INTERRUPT_STACK_SIZE bytes ending at __stack_end__),
 * and until now the trap entry used a literal address:
 *
 *     la    sp, __stack_end__
 *
 * With RT_CPUS_NR == 2 that is a shared resource. A tick on CPU1 while CPU0 is
 * already inside an interrupt would put both cores' frames in the same region,
 * and they grow downward, so they overwrite each other. That is why CPU1 gets no
 * interrupt of its own yet: not a missing timer, but a shared stack.
 *
 * The fix - and why it does not touch IDF's linker script
 * ------------------------------------------------------
 * CPU0 keeps __stack_end__, so the single-core build and CPU0's behaviour are
 * byte-for-byte what they were. CPU1 gets a static array in .bss of the same size.
 * The trap entry then selects a top-of-stack from a table indexed by mhartid,
 * which is exactly the shape rt_irq_nest_slot already uses two instructions
 * earlier - two loads, no call, nothing that can land in flash.
 *
 * Editing sections.ld.in would have meant changing an IDF-owned file to express
 * something the component can express on its own, and it would have made the
 * rollback a linker change instead of a source change.
 *
 * Why the table is plain .data with link-time constants
 * -----------------------------------------------------
 * Same reason as the nest table below: DRAM stays readable with the flash cache
 * off, and the trap handler needs the pointer before any C code has run. Both
 * entries are known at link time, so the .data load places them, and no init call
 * is on the path.
 */

#include <rthw.h>
#include <rtthread.h>

#include "cpuport.h"
#include "esp_attr.h"
#include "sdkconfig.h"

#ifdef RT_USING_SMP

/*
 * CPU1's interrupt stack.
 *
 * In .bss, 16-byte aligned for the RISC-V stack ABI (the same alignment the
 * linker script gives .stack). Sized from the same Kconfig option as CPU0's, so
 * the two cores cannot drift apart in a way that only shows up under load.
 */
static char s31_cpu1_istack[CONFIG_IDF_RTTHREAD_INTERRUPT_STACK_SIZE]
    __attribute__((aligned(16)));

#endif /* RT_USING_SMP */

/*
 * Top of the interrupt stack for each core, indexed by mhartid.
 *
 * Present in both builds: with RT_CPUS_NR == 1 the table has one entry pointing
 * at __stack_end__, which is exactly what the literal `la sp, __stack_end__` used
 * to compute - so the single-core trap path is unchanged.
 */
extern char __stack_end__;

/*
 * Deliberately NOT `void *const`. With const pointers the array can be placed in
 * .flash.rodata, and the trap handler reads it while the flash cache is off -
 * which is exactly the fault this port already fixed twice (rt_irq_nest_slot's
 * header explains both). Non-const keeps it in plain .data next to that table.
 */
void *rt_irq_stack_top[RT_CPUS_NR] =
{
    &__stack_end__,
#ifdef RT_USING_SMP
    s31_cpu1_istack + sizeof(s31_cpu1_istack),
#endif
};
