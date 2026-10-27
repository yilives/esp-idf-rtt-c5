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
 * @param stack_words stack size in StackType_t units (32-bit words), which is
 *                    what FreeRTOS counts in; RT-Thread counts bytes
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

#ifdef __cplusplus
}
#endif

#endif /* __FT_CORE_H__ */
