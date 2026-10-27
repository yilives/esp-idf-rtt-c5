/*
 * FreeRTOS-compatible primitives implemented on RT-Thread.
 *
 * Why this exists: the S31 Wi-Fi and BLE blobs are OS agnostic (verified - see
 * rtthread-esp32-s31-wifi-feasibility.md), but the IDF components around them
 * call 33 FreeRTOS APIs. This layer provides those, backed by rt_* primitives,
 * so those components can run on RT-Thread without being rewritten.
 *
 * Two design rules, both learned from what breaks when ignored:
 *
 * 1. Nothing here includes a FreeRTOS header. The shim has to compile while
 *    libfreertos.a is still linked into the image, so it cannot share type
 *    names or macros with the real kernel. The FreeRTOS names live in
 *    freertos_api.c, which is only built when the kernel is switched off.
 *
 * 2. Every entry point returns an explicit error instead of a bare pdFALSE.
 *    FreeRTOS collapses "full" and "timed out" into the same value, which
 *    makes a failing self test uninformative.
 *
 * Semantics that differ from FreeRTOS on purpose, and why:
 *
 * - xQueueCreateStatic is honoured only when the caller's control block is at
 *   least ft_ipc_size() bytes; otherwise the create fails and returns NULL.
 *   Silently overflowing a StaticQueue_t would be a memory corruption bug that
 *   only shows up minutes later.
 * - Task priorities are mapped, not copied: FreeRTOS counts up, RT-Thread counts
 *   down. See ft_prio_to_rt().
 */

#ifndef __FT_CORE_H__
#define __FT_CORE_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef long            ft_base_t;
typedef unsigned long   ft_ubase_t;
typedef rt_uint32_t     ft_tick_t;

#define ftPASS              ((ft_base_t)1)
#define ftFAIL              ((ft_base_t)0)

/*
 * Bytes per FreeRTOS stack unit.
 *
 * FreeRTOS documents xTaskCreate's stack depth in words, but that is only true
 * when the port defines StackType_t as a 32-bit type. IDF's RISC-V port defines
 * it as uint8_t (portmacro.h:101), so a depth of 2048 means 2048 *bytes*.
 * freertos_api.c asserts sizeof(StackType_t) == this constant, so a port that
 * ever changes its mind breaks the build instead of silently quadrupling every
 * task stack.
 */
#define FT_STACK_UNIT_BYTES 1

/* Timeout arguments, matching portMAX_DELAY / 0. */
#define FT_NO_WAIT          ((ft_tick_t)0)
#define FT_MAX_DELAY        ((ft_tick_t)0xFFFFFFFFu)

/* Return codes. Non-negative is success so the wrappers can do `ret >= 0`. */
#define FT_OK               0
#define FT_ERR_FULL         (-1)
#define FT_ERR_EMPTY        (-2)
#define FT_ERR_TIMEOUT      (-3)
#define FT_ERR_PARAM        (-4)
#define FT_ERR_NOMEM        (-5)
#define FT_ERR_STATE        (-6)

/* Convert a FreeRTOS tick count to an RT-Thread timeout. FT_MAX_DELAY maps to
 * RT_WAITING_FOREVER. Defined once in ft_ipc.c. */
rt_int32_t ft_ticks_to_rt(ft_tick_t ticks);

/**
 * Whether a blocking call with this timeout can work at all.
 *
 * False when the caller is in an ISR, or when the RT-Thread scheduler is locked
 * (which is what an IDF portENTER_CRITICAL leaves behind). Blocking there is
 * guaranteed to strand the caller: rt_schedule() is a no-op while the scheduler
 * lock is held, so the thread leaves the ready queue and is never resumed.
 * Every blocking entry point checks this and returns FT_ERR_STATE instead.
 */
rt_bool_t ft_block_allowed(ft_tick_t ticks, const char *what);

/* ------------------------------------------------------------------ queue -- */

struct ft_ipc;

/**
 * Create a queue of @p length items of @p item_size bytes.
 * @p item_size == 0 gives a semaphore (a queue whose items carry no payload).
 */
struct ft_ipc *ft_ipc_create(ft_ubase_t length, ft_ubase_t item_size);

/**
 * Same, but place the object in the caller's memory.
 *
 * @param storage   ring buffer, length * item_size bytes; may be NULL when
 *                  item_size is 0
 * @param cb        control block, at least ft_ipc_size() bytes
 * @param cb_size   size of @p cb in bytes
 * @return NULL when @p cb is too small
 */
struct ft_ipc *ft_ipc_create_static(ft_ubase_t length, ft_ubase_t item_size,
                                    void *storage, void *cb, ft_ubase_t cb_size);

/**
 * Use the caller's storage and control block, but keep the object itself on
 * the RT-Thread heap.
 *
 * This exists because of xQueueCreateWithCaps(): idf_additions.c allocates a
 * FreeRTOS StaticQueue_t, which is a Queue_t-sized union - far smaller than
 * struct ft_ipc. Placing the object in it would overflow. Instead the caller's
 * storage becomes the ring buffer, its control block is remembered verbatim so
 * xQueueGetStaticBuffers() can hand it back for the caller to free, and the
 * object itself is heap allocated.
 *
 * Deletion frees only the object; the two caller buffers stay the caller's
 * property, which is exactly what vQueueDeleteWithCaps expects.
 */
struct ft_ipc *ft_ipc_create_static_ram(ft_ubase_t length, ft_ubase_t item_size,
                                        void *storage, void *cb);

/**
 * Mutex whose caller-supplied control block is only remembered, not used.
 *
 * xSemaphoreCreateMutexStatic() passes a StaticQueue_t that is too small for
 * struct ft_ipc, and a mutex has no storage area to borrow, so the object goes
 * on the heap and the caller's block is kept for xQueueGetStaticBuffers().
 */
struct ft_ipc *ft_ipc_create_mutex_static(void *cb, ft_base_t recursive);

/** The control block passed to ft_ipc_create_static_ram(), or NULL. */
void *ft_ipc_static_cb(struct ft_ipc *ipc);

/** The ring buffer this object writes into, for xQueueGenericGetStaticBuffers(). */
void *ft_ipc_storage(struct ft_ipc *ipc);

/** Size a caller must provide to ft_ipc_create_static(). */
rt_size_t ft_ipc_size(void);

/** Binary (max 1, initial 0) or counting semaphore. */
struct ft_ipc *ft_ipc_create_sem(ft_ubase_t max_count, ft_ubase_t initial);

/** Mutex with priority inheritance. Not recursive, like xSemaphoreCreateMutex. */
struct ft_ipc *ft_ipc_create_mutex(void);

/**
 * Mutex, recursive or not.
 *
 * @param recursive 0 for xSemaphoreCreateMutex semantics (a second take by the
 *                  owner fails), 1 for xSemaphoreCreateRecursiveMutex
 */
struct ft_ipc *ft_ipc_create_mutex_ex(ft_base_t recursive);

/** Thread currently holding a mutex, or NULL. Used by xQueueGetMutexHolder. */
void *ft_ipc_mutex_holder(struct ft_ipc *ipc);

void ft_ipc_delete(struct ft_ipc *ipc);

/** Post one item. Returns FT_OK, FT_ERR_FULL or FT_ERR_TIMEOUT. */
int ft_ipc_send(struct ft_ipc *ipc, const void *item, ft_tick_t ticks);

/** Non-blocking post. @p woken is set to 1 when a task was made ready. */
int ft_ipc_send_isr(struct ft_ipc *ipc, const void *item, ft_base_t *woken);

/** Fetch one item. Returns FT_OK, FT_ERR_EMPTY or FT_ERR_TIMEOUT. */
int ft_ipc_receive(struct ft_ipc *ipc, void *item, ft_tick_t ticks);

int ft_ipc_receive_isr(struct ft_ipc *ipc, void *item, ft_base_t *woken);

/** Take a semaphore or mutex. */
int ft_ipc_take(struct ft_ipc *ipc, ft_tick_t ticks);

/** Give a semaphore or release a mutex. */
int ft_ipc_give(struct ft_ipc *ipc);

int ft_ipc_give_isr(struct ft_ipc *ipc, ft_base_t *woken);

ft_base_t  ft_ipc_is_empty(struct ft_ipc *ipc);
ft_ubase_t ft_ipc_messages_waiting(struct ft_ipc *ipc);
ft_ubase_t ft_ipc_spaces_available(struct ft_ipc *ipc);

/* ------------------------------------------------------------------ event -- */

struct ft_event;

struct ft_event *ft_event_create(void);
void             ft_event_delete(struct ft_event *event);

/** Set bits; returns the bit set after the update. */
rt_uint32_t ft_event_set_bits(struct ft_event *event, rt_uint32_t bits);

/** Clear bits; returns the bit set after the update. */
rt_uint32_t ft_event_clear_bits(struct ft_event *event, rt_uint32_t bits);

/** Current bit set. */
rt_uint32_t ft_event_get_bits(struct ft_event *event);

/**
 * Wait for any/all of @p bits.
 *
 * @param clear_on_exit clear the matching bits before returning
 * @param wait_for_all  require every bit in @p bits, not just one
 * @return the bit set on success, 0 on timeout or error (FreeRTOS behaviour)
 */
rt_uint32_t ft_event_wait_bits(struct ft_event *event, rt_uint32_t bits,
                               ft_base_t clear_on_exit, ft_base_t wait_for_all,
                               ft_tick_t ticks);

/* ------------------------------------------------------------------- task -- */

/** Convert a FreeRTOS priority to an RT-Thread one. */
rt_uint8_t ft_prio_to_rt(ft_ubase_t prio);

/** Convert back, for uxTaskPriorityGet(). */
ft_ubase_t ft_prio_to_ft(rt_uint8_t prio);

/**
 * Create and start a task.
 *
 * @param stack_words stack size in StackType_t units; on this port one unit is
 *                    one byte (FT_STACK_UNIT_BYTES), rt_thread_create also
 *                    takes bytes
 * @param handle      receives the thread, may be NULL
 */
void *ft_task_create(const char *name, void (*entry)(void *),
                     void *param, ft_ubase_t stack_words,
                     ft_ubase_t prio, void **handle);

void ft_task_delay(ft_tick_t ticks);

/** NULL deletes the calling task, matching vTaskDelete(NULL). */
void ft_task_delete(void *handle);

/** NULL suspends the calling task. */
void ft_task_suspend(void *handle);

void        ft_task_set_priority(void *handle, ft_ubase_t prio);
ft_ubase_t  ft_task_get_priority(void *handle);

void      *ft_task_self(void);
ft_tick_t  ft_task_tick_count(void);

/** Same value; FreeRTOS distinguishes the two only by which context may call. */
ft_tick_t  ft_task_tick_count_isr(void);

/* Scheduler states, matching eSchedulerState in task.h. */
#define FT_SCHEDULER_SUSPENDED      0
#define FT_SCHEDULER_NOT_STARTED    1
#define FT_SCHEDULER_RUNNING        2

/**
 * xTaskGetSchedulerState().
 *
 * RT-Thread has no "not started" state visible from a running thread, so the
 * answer is RUNNING unless the scheduler is locked, which is the case that
 * callers actually branch on (spi_flash and newlib's lock layer use it to
 * decide whether blocking is legal).
 */
ft_base_t ft_task_scheduler_state(void);

/** vTaskSuspendAll / xTaskResumeAll, mapped to the RT-Thread scheduler lock. */
void      ft_task_suspend_all(void);
ft_base_t ft_task_resume_all(void);

void ft_task_set_tls(void *handle, ft_ubase_t index, void *value);
void *ft_task_get_tls(void *handle, ft_ubase_t index);

/**
 * Set a pointer and a callback that runs when the task is deleted.
 *
 * IDF adds this on top of FreeRTOS (idf_additions.h, behind
 * CONFIG_FREERTOS_TLSP_DELETION_CALLBACKS) so a component can hang malloc'ed
 * state off a task and have it released with the task. esp_event uses it for the
 * event loop's per-task bookkeeping; pthread_local_storage.c uses it too.
 *
 * The callback runs from ft_task_delete() and from the thread exit hook, in
 * both cases before the thread object goes away.
 */
typedef void (*ft_tls_del_cb_t)(int index, void *value);

void ft_task_set_tls_with_cb(void *handle, ft_ubase_t index, void *value,
                             ft_tls_del_cb_t cb);

/** Occupied slots. For the self test: the release path is otherwise invisible. */
rt_uint32_t ft_tls_slots_used(void);

/* ------------------------------------------------- task introspection ----- */

/*
 * Declared unconditionally by task.h, defined only by tasks.c, which is not
 * compiled when the shim is on. Defining them here is not optional: the first
 * caller to use one fails at link time, in a file whose author had no way to
 * know the shim was involved.
 */

/** uxTaskGetNumberOfTasks(). Includes idle. */
ft_ubase_t ft_task_count(void);

/** pcTaskGetName(). NULL means the caller. */
const char *ft_task_name(void *handle);

/** xTaskGetHandle(): look a thread up by name. */
void *ft_task_find(const char *name);

/** xTaskGetIdleTaskHandle(). */
void *ft_task_idle_handle(void);

/** eTaskGetState(). Returns the numeric value of the eTaskState enum. */
rt_uint32_t ft_task_state(void *handle);

/** xTaskAbortDelay(). */
ft_base_t ft_task_abort_delay(void *handle);

/** xTaskResumeFromISR(). */
ft_base_t ft_task_resume_isr(void *handle, ft_base_t *woken);

/** xTaskDelayUntil(). @p prev is in FreeRTOS ticks and is advanced in place. */
ft_base_t ft_task_delay_until(ft_tick_t *prev, ft_tick_t increment);

/* ------------------------------------------------------- task notification -- */

/*
 * Task notifications, used as a per-task binary/counting wakeup with an
 * optional bit payload.
 *
 * Why it matters: stream_buffer.c (which esp_driver_uart reaches through
 * esp_ringbuf) does its whole blocking protocol with
 * xTaskNotify(eNoAction) + xTaskNotifyWait, and esp_timer/esp_eth/esp_ipc use
 * ulTaskNotifyTake. Without these, neither stream buffers nor timers can block.
 *
 * The value semantics below cover eNoAction, eSetBits, eIncrement and both
 * eSetValue forms. eSetValueWithoutOverwrite returns FT_ERR_STATE when a
 * notification is still pending, which is what FreeRTOS reports as pdFAIL.
 */

#define FT_NOTIFY_NO_ACTION             0
#define FT_NOTIFY_SET_BITS              1
#define FT_NOTIFY_INCREMENT             2
#define FT_NOTIFY_SET_VALUE_OVERWRITE   3
#define FT_NOTIFY_SET_VALUE_NO_OVERWRITE 4

/** xTaskNotify(). Returns FT_OK or FT_ERR_STATE. */
int ft_task_notify(void *handle, rt_uint32_t value, ft_base_t action);

/** xTaskNotifyFromISR(). @p woken set to 1 if a task was made ready. */
int ft_task_notify_isr(void *handle, rt_uint32_t value, ft_base_t action,
                       ft_base_t *woken);

/**
 * xTaskNotifyWait().
 *
 * @param clear_on_entry clear the value before blocking
 * @param clear_on_exit  clear the value being returned
 * @param timed_out      optional; set to 1 when the wait expired
 * @return the notification value (0 when @p clear_on_entry cleared it and
 *         nothing new arrived, matching FreeRTOS)
 */
rt_uint32_t ft_task_notify_wait(rt_uint32_t clear_on_entry,
                                rt_uint32_t clear_on_exit,
                                ft_tick_t ticks, ft_base_t *timed_out);

/** xTaskNotifyStateClear(). Returns the value that was pending. */
rt_uint32_t ft_task_notify_clear(void *handle);

/** Current notification value without changing it (xTaskNotify's out param). */
rt_uint32_t ft_task_notify_peek(void *handle);

/**
 * ulTaskNotifyTake().
 *
 * @param clear_on_exit pdTRUE for binary semantics (reset the count to 0 and
 *                      return what was there), pdFALSE to decrement by one
 * @return the count before the take
 */
rt_uint32_t ft_task_notify_take(ft_base_t clear_on_exit, ft_tick_t ticks);

/**
 * Drop a thread's notification slot.
 *
 * Must be called before the thread object is freed, because the slot is keyed
 * by the thread pointer and rt_thread_create() reuses addresses. ft_task.c's
 * exit hook does this.
 */
void ft_notify_forget(rt_thread_t thread);

/* ------------------------------------------------------------- timeouts -- */

/*
 * Mirrors TimeOut_t, field for field, so freertos_api.c can cast between them.
 * BaseType_t is long and TickType_t is uint32_t on this port, which is what
 * ft_base_t and ft_tick_t are.
 */
typedef struct
{
    ft_base_t xOverflowCount;
    ft_tick_t xTimeOnEntering;
} ft_timeout_t;

void      ft_timeout_set(ft_timeout_t *timeout);
ft_base_t ft_timeout_check(ft_timeout_t *timeout, ft_tick_t *remaining);

#ifdef __cplusplus
}
#endif

#endif /* __FT_CORE_H__ */
