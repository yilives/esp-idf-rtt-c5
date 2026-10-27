/*
 * Queues, semaphores and mutexes on RT-Thread.
 *
 * FreeRTOS builds all three from one object: a queue has length + item size,
 * a binary semaphore is a length-1 queue with no payload, a counting semaphore
 * is a length-N queue with no payload, and a mutex is a separate type with
 * priority inheritance. The same shape is mirrored here so that
 * xQueueSemaphoreTake() can accept any of them and dispatch on `kind`.
 *
 * Blocking uses two RT-Thread semaphores per object: `available` counts stored
 * items, `free_slots` counts room. A sender must first take a free slot, so N
 * concurrent senders can never collide on the ring index; the index update
 * itself is done under an interrupt lock because an ISR can push at any time.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "ft_core.h"

enum ft_ipc_kind
{
    FT_IPC_KIND_QUEUE = 1,
    FT_IPC_KIND_SEM,
    FT_IPC_KIND_MUTEX,      /* non-recursive, like xSemaphoreCreateMutex */
    FT_IPC_KIND_RECMUTEX,   /* recursive, like xSemaphoreCreateRecursiveMutex */
};

struct ft_ipc
{
    rt_uint8_t  kind;
    rt_uint8_t  owns_memory;

    rt_uint8_t *buf;
    rt_uint32_t item_size;
    rt_uint32_t length;
    rt_uint32_t head;           /* next slot to write */
    rt_uint32_t tail;           /* next slot to read */
    rt_uint32_t count;          /* stored items, authoritative for the queries */

    struct rt_semaphore available;
    struct rt_semaphore free_slots;

    struct rt_mutex mutex;      /* both mutex kinds */

    /* RT-Thread mutexes are recursive by nature: rt_mutex_take by the owner
     * just bumps hold. FreeRTOS's plain mutex is not, and code relies on that,
     * so the plain kind is guarded here and only the recursive kind is allowed
     * to use rt_mutex's own recursion. */
    rt_thread_t owner;          /* FT_IPC_KIND_MUTEX only */

    /* Set when the object was heap allocated by ft_ipc_create_static_ram()
     * because the caller's control block (a FreeRTOS StaticQueue_t) is too
     * small to hold struct ft_ipc. Kept so xQueueGetStaticBuffers() can hand
     * the caller's own block back for it to free. */
    void       *static_cb;
};

/* FreeRTOS ticks to RT-Thread ticks. Both are 1 kHz in this configuration, but
 * the conversion is written out so changing either one cannot silently scale
 * every timeout in the system. */
rt_int32_t ft_ticks_to_rt(ft_tick_t ticks)
{
    if (ticks == FT_MAX_DELAY)
    {
        return RT_WAITING_FOREVER;
    }

    return (rt_int32_t)(((rt_uint64_t)ticks * RT_TICK_PER_SECOND) / CONFIG_FREERTOS_HZ);
}

rt_size_t ft_ipc_size(void)
{
    return sizeof(struct ft_ipc);
}

/*
 * Refuse to block when the RT-Thread scheduler is locked, or from an ISR.
 *
 * This is not defensive coding; it turns a hang into an error. rt_schedule()
 * does nothing at all while rt_scheduler_lock_nest is non-zero
 * (src/scheduler_up.c:296), so _rt_sem_take() suspends the caller, fails to
 * switch away, and returns -RT_EINTR while the thread is still on the
 * semaphore's suspend list (src/ipc.c sets thread->error = RT_EINTR before the
 * suspend and only the resumer clears it). The caller then keeps running with
 * its status already set to suspend and its ready-queue entry gone, so the next
 * reschedule parks it forever with no trace.
 *
 * FreeRTOS enforces the same rule with configASSERT in vTaskPlaceOnEventList.
 * The difference is that an assert stops the board at the call site, which is
 * what we want here too - but recoverable, because some IDF paths probe for it.
 */
rt_bool_t ft_block_allowed(ft_tick_t ticks, const char *what)
{
    static rt_uint32_t warned;

    if (ticks == 0)
    {
        return RT_TRUE;         /* non-blocking is always legal */
    }

    if (rt_interrupt_get_nest() != 0)
    {
        if (!warned)
        {
            warned = 1;
            rt_kprintf("freertos-shim: %s cannot block in an ISR\n", what);
        }
        return RT_FALSE;
    }

    if (rt_critical_level() != 0)
    {
        if (!warned)
        {
            warned = 1;
            rt_kprintf("freertos-shim: %s cannot block with the scheduler locked "
                       "(an IDF critical section is held)\n", what);
        }
        return RT_FALSE;
    }

    return RT_TRUE;
}

static struct ft_ipc *ft_ipc_init(struct ft_ipc *q, rt_uint8_t kind,
                                  ft_ubase_t length, ft_ubase_t item_size,
                                  rt_uint8_t *storage, rt_bool_t owns)
{
    rt_memset(q, 0, sizeof(*q));

    q->kind        = kind;
    q->owns_memory = owns ? 1 : 0;
    q->buf         = storage;
    q->item_size   = item_size;
    q->length      = length ? (rt_uint32_t)length : 1u;

    if (kind == FT_IPC_KIND_MUTEX || kind == FT_IPC_KIND_RECMUTEX)
    {
        if (rt_mutex_init(&q->mutex, "ftmtx", RT_IPC_FLAG_PRIO) != RT_EOK)
        {
            return RT_NULL;
        }
        return q;
    }

    if (rt_sem_init(&q->available, "ftav", 0, RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        return RT_NULL;
    }

    if (rt_sem_init(&q->free_slots, "ftfs", q->length, RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        rt_sem_detach(&q->available);
        return RT_NULL;
    }

    return q;
}

struct ft_ipc *ft_ipc_create(ft_ubase_t length, ft_ubase_t item_size)
{
    struct ft_ipc *q;
    rt_uint8_t    *buf = RT_NULL;

    if (length == 0)
    {
        return RT_NULL;
    }

    /* ft_ubase_t is unsigned long, which is 32 bits here, so
     * length * item_size can wrap. A 65536-entry queue of 65536-byte items
     * would ask rt_malloc for 0 bytes, succeed, and then let the first send
     * write past the end of a zero-length buffer - a heap corruption whose
     * report arrives somewhere unrelated. FreeRTOS rejects the same combination
     * by returning NULL from xQueueGenericCreate.
     *
     * RT_SIZE_MAX rather than SIZE_MAX: rtdef.h picks the right width for the
     * target, and it is already on the include path. */
    if (item_size != 0 && length > (ft_ubase_t)(RT_SIZE_MAX / item_size))
    {
        return RT_NULL;
    }

    q = rt_malloc(sizeof(*q));
    if (q == RT_NULL)
    {
        return RT_NULL;
    }

    if (item_size != 0)
    {
        buf = rt_malloc((rt_size_t)length * item_size);
        if (buf == RT_NULL)
        {
            rt_free(q);
            return RT_NULL;
        }
    }

    if (ft_ipc_init(q, item_size ? FT_IPC_KIND_QUEUE : FT_IPC_KIND_SEM,
                    length, item_size, buf, RT_TRUE) == RT_NULL)
    {
        if (buf)
        {
            rt_free(buf);
        }
        rt_free(q);
        return RT_NULL;
    }

    return q;
}

struct ft_ipc *ft_ipc_create_static(ft_ubase_t length, ft_ubase_t item_size,
                                    void *storage, void *cb, ft_ubase_t cb_size)
{
    /* Refusing here is the whole point of the size check: writing a 160-byte
     * object into a caller's StaticQueue_t would corrupt whatever follows it,
     * and the symptom would appear somewhere unrelated minutes later. */
    if (cb == RT_NULL || cb_size < sizeof(struct ft_ipc) || length == 0)
    {
        return RT_NULL;
    }

    return ft_ipc_init((struct ft_ipc *)cb,
                       item_size ? FT_IPC_KIND_QUEUE : FT_IPC_KIND_SEM,
                       length, item_size, (rt_uint8_t *)storage, RT_FALSE);
}

struct ft_ipc *ft_ipc_create_static_ram(ft_ubase_t length, ft_ubase_t item_size,
                                        void *storage, void *cb)
{
    struct ft_ipc *q;

    if (cb == RT_NULL || length == 0)
    {
        return RT_NULL;
    }

    q = rt_malloc(sizeof(*q));
    if (q == RT_NULL)
    {
        return RT_NULL;
    }

    if (ft_ipc_init(q, item_size ? FT_IPC_KIND_QUEUE : FT_IPC_KIND_SEM,
                    length, item_size, (rt_uint8_t *)storage, RT_FALSE) == RT_NULL)
    {
        rt_free(q);
        return RT_NULL;
    }

    q->static_cb = cb;

    return q;
}

void *ft_ipc_static_cb(struct ft_ipc *ipc)
{
    return (ipc != RT_NULL) ? ipc->static_cb : RT_NULL;
}

struct ft_ipc *ft_ipc_create_mutex_static(void *cb, ft_base_t recursive)
{
    struct ft_ipc *q;

    q = rt_malloc(sizeof(*q));
    if (q == RT_NULL)
    {
        return RT_NULL;
    }

    if (ft_ipc_init(q, recursive ? FT_IPC_KIND_RECMUTEX : FT_IPC_KIND_MUTEX,
                    1, 0, RT_NULL, RT_FALSE) == RT_NULL)
    {
        rt_free(q);
        return RT_NULL;
    }

    q->static_cb = cb;

    return q;
}

void *ft_ipc_storage(struct ft_ipc *ipc)
{
    return (ipc != RT_NULL) ? ipc->buf : RT_NULL;
}

struct ft_ipc *ft_ipc_create_sem(ft_ubase_t max_count, ft_ubase_t initial)
{
    struct ft_ipc *q;
    ft_ubase_t i;

    q = ft_ipc_create(max_count ? max_count : 1, 0);
    if (q == RT_NULL)
    {
        return RT_NULL;
    }

    for (i = 0; i < initial; i++)
    {
        if (ft_ipc_give(q) != FT_OK)
        {
            break;
        }
    }

    return q;
}

struct ft_ipc *ft_ipc_create_mutex_ex(ft_base_t recursive)
{
    struct ft_ipc *q = rt_malloc(sizeof(*q));

    if (q == RT_NULL)
    {
        return RT_NULL;
    }

    if (ft_ipc_init(q, recursive ? FT_IPC_KIND_RECMUTEX : FT_IPC_KIND_MUTEX,
                    1, 0, RT_NULL, RT_TRUE) == RT_NULL)
    {
        rt_free(q);
        return RT_NULL;
    }

    return q;
}

struct ft_ipc *ft_ipc_create_mutex(void)
{
    return ft_ipc_create_mutex_ex(0);
}

void *ft_ipc_mutex_holder(struct ft_ipc *ipc)
{
    if (ipc == RT_NULL)
    {
        return RT_NULL;
    }

    if (ipc->kind == FT_IPC_KIND_MUTEX)
    {
        return ipc->owner;
    }

    if (ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        return ipc->mutex.owner;
    }

    return RT_NULL;
}

void ft_ipc_delete(struct ft_ipc *ipc)
{
    if (ipc == RT_NULL)
    {
        return;
    }

    if (ipc->kind == FT_IPC_KIND_MUTEX || ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        rt_mutex_detach(&ipc->mutex);
    }
    else
    {
        rt_sem_detach(&ipc->available);
        rt_sem_detach(&ipc->free_slots);
    }

    if (ipc->owns_memory)
    {
        if (ipc->buf)
        {
            rt_free(ipc->buf);
        }
        rt_free(ipc);
    }
    else if (ipc->static_cb != RT_NULL)
    {
        /* Object is ours, the storage and control block belong to the caller.
         * vQueueDeleteWithCaps() frees those two itself. */
        ipc->static_cb = RT_NULL;
        rt_free(ipc);
    }
    else
    {
        rt_memset(ipc, 0, sizeof(*ipc));
    }
}

/* Ring update. Short and interrupt-locked: taking a free slot already
 * guarantees the slot exists, this only makes the index arithmetic atomic with
 * respect to an ISR pushing at the same time. */
static void ft_ipc_push(struct ft_ipc *q, const void *item)
{
    rt_base_t level = rt_hw_interrupt_disable();

    if (q->item_size != 0 && item != RT_NULL)
    {
        rt_memcpy(q->buf + q->head * q->item_size, item, q->item_size);
    }

    q->head = (q->head + 1u) % q->length;
    q->count++;

    rt_hw_interrupt_enable(level);
}

static void ft_ipc_pop(struct ft_ipc *q, void *item)
{
    rt_base_t level = rt_hw_interrupt_disable();

    if (q->item_size != 0 && item != RT_NULL)
    {
        rt_memcpy(item, q->buf + q->tail * q->item_size, q->item_size);
    }

    q->tail = (q->tail + 1u) % q->length;
    q->count--;

    rt_hw_interrupt_enable(level);
}

int ft_ipc_send(struct ft_ipc *ipc, const void *item, ft_tick_t ticks)
{
    if (ipc == RT_NULL)
    {
        return FT_ERR_PARAM;
    }

    if (ipc->kind == FT_IPC_KIND_MUTEX || ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        return ft_ipc_give(ipc);
    }

    /* Try the non-blocking path first. A free slot means this call cannot
     * block, so it stays legal inside a critical section - FreeRTOS's assert
     * guards vTaskPlaceOnEventList, i.e. the blocking path only. */
    if (rt_sem_trytake(&ipc->free_slots) != RT_EOK)
    {
        if (ticks == 0)
        {
            return FT_ERR_FULL;
        }

        if (!ft_block_allowed(ticks, "xQueueSend"))
        {
            return FT_ERR_STATE;
        }

        if (rt_sem_take(&ipc->free_slots, ft_ticks_to_rt(ticks)) != RT_EOK)
        {
            return FT_ERR_FULL;
        }
    }

    ft_ipc_push(ipc, item);
    rt_sem_release(&ipc->available);

    return FT_OK;
}

int ft_ipc_send_isr(struct ft_ipc *ipc, const void *item, ft_base_t *woken)
{
    if (woken != RT_NULL)
    {
        *woken = ftFAIL;
    }

    if (ipc == RT_NULL)
    {
        return FT_ERR_PARAM;
    }

    if (ipc->kind == FT_IPC_KIND_MUTEX || ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        return ft_ipc_give_isr(ipc, woken);
    }

    if (rt_sem_trytake(&ipc->free_slots) != RT_EOK)
    {
        return FT_ERR_FULL;
    }

    ft_ipc_push(ipc, item);
    rt_sem_release(&ipc->available);

    /* Conservative: RT-Thread's release does not report whether a waiter was
     * unblocked, so always ask for a reschedule. An unnecessary yield only
     * costs a scheduler pass, while a missing one delays a higher priority
     * task by a full tick. */
    if (woken != RT_NULL)
    {
        *woken = ftPASS;
    }

    return FT_OK;
}

static int ft_ipc_take_item(struct ft_ipc *ipc, void *item, ft_tick_t ticks)
{
    if (rt_sem_trytake(&ipc->available) != RT_EOK)
    {
        if (ticks == 0)
        {
            return FT_ERR_TIMEOUT;
        }

        if (!ft_block_allowed(ticks, "xQueueReceive"))
        {
            return FT_ERR_STATE;
        }

        if (rt_sem_take(&ipc->available, ft_ticks_to_rt(ticks)) != RT_EOK)
        {
            return FT_ERR_TIMEOUT;
        }
    }

    ft_ipc_pop(ipc, item);
    rt_sem_release(&ipc->free_slots);

    return FT_OK;
}

int ft_ipc_receive(struct ft_ipc *ipc, void *item, ft_tick_t ticks)
{
    if (ipc == RT_NULL ||
        ipc->kind == FT_IPC_KIND_MUTEX || ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        return FT_ERR_PARAM;
    }

    return ft_ipc_take_item(ipc, item, ticks);
}

int ft_ipc_receive_isr(struct ft_ipc *ipc, void *item, ft_base_t *woken)
{
    if (woken != RT_NULL)
    {
        *woken = ftFAIL;
    }

    if (ipc == RT_NULL ||
        ipc->kind == FT_IPC_KIND_MUTEX || ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        return FT_ERR_PARAM;
    }

    if (rt_sem_trytake(&ipc->available) != RT_EOK)
    {
        return FT_ERR_EMPTY;
    }

    ft_ipc_pop(ipc, item);
    rt_sem_release(&ipc->free_slots);

    if (woken != RT_NULL)
    {
        *woken = ftPASS;
    }

    return FT_OK;
}

int ft_ipc_take(struct ft_ipc *ipc, ft_tick_t ticks)
{
    if (ipc == RT_NULL)
    {
        return FT_ERR_PARAM;
    }

    if (ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        /* rt_mutex_take already lets the owner take it again, and for the
         * owner that path never blocks. */
        if (rt_mutex_trytake(&ipc->mutex) == RT_EOK)
        {
            return FT_OK;
        }

        if (ticks == 0)
        {
            return FT_ERR_TIMEOUT;
        }

        if (!ft_block_allowed(ticks, "xSemaphoreTake"))
        {
            return FT_ERR_STATE;
        }

        return (rt_mutex_take(&ipc->mutex, ft_ticks_to_rt(ticks)) == RT_EOK)
               ? FT_OK : FT_ERR_TIMEOUT;
    }

    if (ipc->kind == FT_IPC_KIND_MUTEX)
    {
        rt_thread_t self = rt_thread_self();

        /* FreeRTOS would deadlock here; failing is the only useful answer. */
        if (ipc->owner == self)
        {
            return FT_ERR_TIMEOUT;
        }

        if (rt_mutex_trytake(&ipc->mutex) == RT_EOK)
        {
            ipc->owner = self;
            return FT_OK;
        }

        if (ticks == 0)
        {
            return FT_ERR_TIMEOUT;
        }

        if (!ft_block_allowed(ticks, "xSemaphoreTake"))
        {
            return FT_ERR_STATE;
        }

        if (rt_mutex_take(&ipc->mutex, ft_ticks_to_rt(ticks)) != RT_EOK)
        {
            return FT_ERR_TIMEOUT;
        }

        ipc->owner = self;
        return FT_OK;
    }

    return ft_ipc_take_item(ipc, RT_NULL, ticks);
}

int ft_ipc_give(struct ft_ipc *ipc)
{
    if (ipc == RT_NULL)
    {
        return FT_ERR_PARAM;
    }

    if (ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        return (rt_mutex_release(&ipc->mutex) == RT_EOK) ? FT_OK : FT_ERR_STATE;
    }

    if (ipc->kind == FT_IPC_KIND_MUTEX)
    {
        if (ipc->owner != rt_thread_self())
        {
            return FT_ERR_STATE;    /* only the owner may release */
        }

        if (rt_mutex_release(&ipc->mutex) != RT_EOK)
        {
            return FT_ERR_STATE;
        }

        ipc->owner = RT_NULL;
        return FT_OK;
    }

    return ft_ipc_send(ipc, RT_NULL, FT_NO_WAIT);
}

int ft_ipc_give_isr(struct ft_ipc *ipc, ft_base_t *woken)
{
    if (ipc != RT_NULL &&
        (ipc->kind == FT_IPC_KIND_MUTEX || ipc->kind == FT_IPC_KIND_RECMUTEX))
    {
        /* A mutex cannot be released from an ISR in FreeRTOS either. */
        if (woken != RT_NULL)
        {
            *woken = ftFAIL;
        }
        return FT_ERR_PARAM;
    }

    return ft_ipc_send_isr(ipc, RT_NULL, woken);
}

ft_base_t ft_ipc_is_empty(struct ft_ipc *ipc)
{
    if (ipc == RT_NULL)
    {
        return ftPASS;
    }

    return ipc->count ? ftFAIL : ftPASS;
}

ft_ubase_t ft_ipc_messages_waiting(struct ft_ipc *ipc)
{
    if (ipc == RT_NULL ||
        ipc->kind == FT_IPC_KIND_MUTEX || ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        return 0;
    }

    return ipc->count;
}

ft_ubase_t ft_ipc_spaces_available(struct ft_ipc *ipc)
{
    if (ipc == RT_NULL ||
        ipc->kind == FT_IPC_KIND_MUTEX || ipc->kind == FT_IPC_KIND_RECMUTEX)
    {
        return 0;
    }

    return (ipc->length > ipc->count) ? (ipc->length - ipc->count) : 0;
}
