/*
 * Tasks on RT-Thread.
 *
 * The mapping is mostly mechanical, with three traps:
 *
 * 1. Priorities run in opposite directions. FreeRTOS counts up (higher value =
 *    more important), RT-Thread counts down. Both conversions live in
 *    ft_prio_to_rt()/ft_prio_to_ft() so the sign can never be flipped in one
 *    place and not another.
 *
 * 2. Stack sizes. FreeRTOS documents xTaskCreate's depth in words, but IDF's
 *    RISC-V port defines StackType_t as uint8_t, so the unit here is a byte and
 *    rt_thread_create wants bytes too - the conversion factor is
 *    FT_STACK_UNIT_BYTES (1). Getting this from the documentation instead of
 *    from portmacro.h gives a task four times the stack it asked for, which is
 *    quiet and wasteful rather than fatal, and that is exactly why it survives
 *    review.
 *
 * 3. rt_thread_control(RT_THREAD_CTRL_CHANGE_PRIORITY) dereferences its
 *    argument as a pointer to rt_uint8_t (src/thread.c:845), so the value has
 *    to be passed by address, not cast to void *.
 *
 * Thread local storage has no RT-Thread equivalent, so a small fixed side table
 * keyed by thread pointer stands in. configNUM_THREAD_LOCAL_STORAGE_POINTERS is
 * 2 in this configuration (CONFIG_FREERTOS_THREAD_LOCAL_STORAGE_POINTERS * 2),
 * and the table is sized well past any plausible task count; a thread that does
 * not fit simply gets NULL back rather than corrupting someone else's slot.
 */

#include <rtthread.h>

#include "sdkconfig.h"
#include "ft_core.h"

#define FT_MAX_PRIORITIES   25      /* configMAX_PRIORITIES */
#define FT_TASK_TIMESLICE   10
#define FT_TLS_SLOTS        (CONFIG_FREERTOS_THREAD_LOCAL_STORAGE_POINTERS * 2)
#define FT_TLS_THREADS      16

rt_uint8_t ft_prio_to_rt(ft_ubase_t prio)
{
    if (prio >= FT_MAX_PRIORITIES)
    {
        prio = FT_MAX_PRIORITIES - 1;
    }

    /* ft 0  -> RT_THREAD_PRIORITY_MAX - 1 (lowest)
     * ft 24 -> RT_THREAD_PRIORITY_MAX - 25 (highest of the mapped band) */
    return (rt_uint8_t)(RT_THREAD_PRIORITY_MAX - 1u - prio);
}

ft_ubase_t ft_prio_to_ft(rt_uint8_t prio)
{
    if (prio >= RT_THREAD_PRIORITY_MAX)
    {
        prio = RT_THREAD_PRIORITY_MAX - 1;
    }

    return (ft_ubase_t)(RT_THREAD_PRIORITY_MAX - 1u - prio);
}

void *ft_task_create(const char *name, void (*entry)(void *), void *param,
                     ft_ubase_t stack_words, ft_ubase_t prio, void **handle)
{
    rt_uint32_t stack_bytes = (rt_uint32_t)stack_words * FT_STACK_UNIT_BYTES;
    rt_thread_t thread;

    if (handle != RT_NULL)
    {
        *handle = RT_NULL;
    }

    if (entry == RT_NULL || stack_bytes < 256)
    {
        return RT_NULL;
    }

    thread = rt_thread_create(name ? name : "fttask", entry, param,
                              stack_bytes, ft_prio_to_rt(prio),
                              FT_TASK_TIMESLICE);
    if (thread == RT_NULL)
    {
        return RT_NULL;
    }

    if (rt_thread_startup(thread) != RT_EOK)
    {
        rt_thread_delete(thread);
        return RT_NULL;
    }

    if (handle != RT_NULL)
    {
        *handle = thread;
    }

    return thread;
}

void ft_task_delay(ft_tick_t ticks)
{
    if (ticks == 0)
    {
        /* FreeRTOS vTaskDelay(0) is a yield, not a no-op. */
        rt_thread_yield();
        return;
    }

    rt_thread_mdelay((rt_int32_t)(((rt_uint64_t)ticks * 1000u) / CONFIG_FREERTOS_HZ));
}

/* Defined with the TLS table further down; declared here because vTaskDelete
 * has to release the per-slot delete callbacks before the thread disappears. */
static void ft_tls_release(rt_thread_t thread);

void ft_task_delete(void *handle)
{
    rt_thread_t thread = handle ? (rt_thread_t)handle : rt_thread_self();

    /* FreeRTOS runs the TLSP deletion callbacks from prvDeleteTCB(), i.e. as
     * part of the delete. IDF's esp_event relies on that to free the per-task
     * state it hangs off index 0. */
    ft_tls_release(thread);

    /* RT-Thread 5.x has no public rt_thread_exit(); rt_thread_delete() accepts
     * the calling thread and hands it to the reaper (src/thread.c:614 routes
     * through _thread_detach), which is what vTaskDelete(NULL) means. */
    rt_thread_delete(thread);
}

void ft_task_suspend(void *handle)
{
    rt_thread_t thread = handle ? (rt_thread_t)handle : rt_thread_self();

    if (rt_thread_suspend(thread) == RT_EOK && thread == rt_thread_self())
    {
        rt_schedule();
    }
}

void ft_task_set_priority(void *handle, ft_ubase_t prio)
{
    rt_thread_t thread = handle ? (rt_thread_t)handle : rt_thread_self();
    rt_uint8_t  new_prio = ft_prio_to_rt(prio);

    /* By address: rt_thread_control dereferences the argument. */
    rt_thread_control(thread, RT_THREAD_CTRL_CHANGE_PRIORITY, &new_prio);
}

ft_ubase_t ft_task_get_priority(void *handle)
{
    rt_thread_t thread = handle ? (rt_thread_t)handle : rt_thread_self();

    return ft_prio_to_ft(RT_SCHED_PRIV(thread).current_priority);
}

void *ft_task_self(void)
{
    return rt_thread_self();
}

ft_tick_t ft_task_tick_count(void)
{
    return (ft_tick_t)rt_tick_get();
}

/* ------------------------------------------------------------------- TLS -- */

struct ft_tls_entry
{
    rt_thread_t thread;
    void       *value[FT_TLS_SLOTS];
    ft_tls_del_cb_t cb[FT_TLS_SLOTS];
};

static struct ft_tls_entry s_tls[FT_TLS_THREADS];

static struct ft_tls_entry *ft_tls_lookup(rt_thread_t thread, rt_bool_t create)
{
    struct ft_tls_entry *free_entry = RT_NULL;
    int i;

    if (thread == RT_NULL)
    {
        return RT_NULL;
    }

    for (i = 0; i < FT_TLS_THREADS; i++)
    {
        if (s_tls[i].thread == thread)
        {
            return &s_tls[i];
        }
        if (free_entry == RT_NULL && s_tls[i].thread == RT_NULL)
        {
            free_entry = &s_tls[i];
        }
    }

    if (!create || free_entry == RT_NULL)
    {
        return RT_NULL;
    }

    rt_memset(free_entry->value, 0, sizeof(free_entry->value));
    rt_memset(free_entry->cb, 0, sizeof(free_entry->cb));
    free_entry->thread = thread;

    return free_entry;
}

void ft_task_set_tls(void *handle, ft_ubase_t index, void *value)
{
    rt_thread_t thread = handle ? (rt_thread_t)handle : rt_thread_self();
    rt_base_t   level;
    struct ft_tls_entry *entry;

    if (index >= FT_TLS_SLOTS)
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    entry = ft_tls_lookup(thread, RT_TRUE);
    if (entry != RT_NULL)
    {
        entry->value[index] = value;
    }
    rt_hw_interrupt_enable(level);
}

void ft_task_set_tls_with_cb(void *handle, ft_ubase_t index, void *value,
                             ft_tls_del_cb_t cb)
{
    rt_thread_t thread = handle ? (rt_thread_t)handle : rt_thread_self();
    rt_base_t   level;
    struct ft_tls_entry *entry;

    if (index >= FT_TLS_SLOTS)
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    entry = ft_tls_lookup(thread, RT_TRUE);
    if (entry != RT_NULL)
    {
        entry->value[index] = value;
        entry->cb[index]    = cb;
    }
    rt_hw_interrupt_enable(level);
}

/* Runs the callbacks and drops the slot. The callbacks are invoked with
 * interrupts enabled: they are component code and may malloc/free. */
static void ft_tls_release(rt_thread_t thread)
{
    struct ft_tls_entry *entry;
    ft_tls_del_cb_t      cb;
    void                *value;
    rt_base_t            level;
    int                  i;

    level = rt_hw_interrupt_disable();
    entry = ft_tls_lookup(thread, RT_FALSE);

    if (entry == RT_NULL)
    {
        rt_hw_interrupt_enable(level);
        return;
    }

    for (i = 0; i < FT_TLS_SLOTS; i++)
    {
        if (entry->value[i] == RT_NULL || entry->cb[i] == RT_NULL)
        {
            continue;
        }

        cb    = entry->cb[i];
        value = entry->value[i];

        entry->value[i] = RT_NULL;
        entry->cb[i]    = RT_NULL;

        rt_hw_interrupt_enable(level);
        cb(i, value);
        level = rt_hw_interrupt_disable();
    }

    entry->thread = RT_NULL;
    rt_hw_interrupt_enable(level);
}

void *ft_task_get_tls(void *handle, ft_ubase_t index)
{
    rt_thread_t thread = handle ? (rt_thread_t)handle : rt_thread_self();
    rt_base_t   level;
    struct ft_tls_entry *entry;
    void *value = RT_NULL;

    if (index >= FT_TLS_SLOTS)
    {
        return RT_NULL;
    }

    level = rt_hw_interrupt_disable();
    entry = ft_tls_lookup(thread, RT_FALSE);
    if (entry != RT_NULL)
    {
        value = entry->value[index];
    }
    rt_hw_interrupt_enable(level);

    return value;
}

ft_tick_t ft_task_tick_count_isr(void)
{
    return (ft_tick_t)rt_tick_get();
}

ft_base_t ft_task_scheduler_state(void)
{
    /* FreeRTOS reports SUSPENDED while vTaskSuspendAll is in effect, and callers
     * use that to decide whether they may block. rt_critical_level() counts the
     * same nesting for rt_enter_critical(), so it is the right source. The
     * NOT_STARTED state is unreachable from a running thread. */
    return rt_critical_level() ? FT_SCHEDULER_SUSPENDED : FT_SCHEDULER_RUNNING;
}

void ft_task_suspend_all(void)
{
    rt_enter_critical();
}

ft_base_t ft_task_resume_all(void)
{
    rt_exit_critical();

    /* FreeRTOS returns pdTRUE when a switch is pending so the caller can
     * yield immediately. RT-Thread's exit already reschedules when needed, so
     * the value is only a hint; answering "yes" costs at most one extra
     * scheduler pass and never delays a waiting task. */
    return ftPASS;
}
