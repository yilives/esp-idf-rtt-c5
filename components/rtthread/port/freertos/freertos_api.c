/*
 * FreeRTOS API surface, mapped onto the ft_* layer.
 *
 * This file only exists when CONFIG_IDF_RTOS_RTTHREAD_FREERTOS_SHIM is on, i.e.
 * when components/freertos no longer compiles queue.c / tasks.c / timers.c /
 * event_groups.c / stream_buffer.c. Defining xQueueCreate while the real kernel
 * is still in the link would be a lottery: both archives would define it and
 * the winner would depend on archive order.
 *
 * Everything here is a one or two line delegation. If something in this file
 * starts containing logic, the logic belongs in ft_*.c instead.
 *
 * Two deviations from FreeRTOS that are deliberate:
 *
 * 1. Static allocation keeps the caller's storage but heap-allocates the
 *    object. ft_ipc is about 170 bytes and StaticQueue_t is about 120, so a
 *    caller's control block can never hold the object. Overflowing it would
 *    corrupt whatever follows, and failing would break esp_libc's lock
 *    initialisation, so ft_ipc_create_static_ram() uses the caller's storage as
 *    the ring buffer, remembers its control block verbatim for
 *    xQueueGenericGetStaticBuffers() to hand back, and keeps only the object on
 *    the heap. Deletion frees the object; the caller frees its own two buffers,
 *    which is exactly what vQueueDeleteWithCaps() does.
 *
 * 2. vTaskSwitchContext() is a trap. It is the function the FreeRTOS port calls
 *    to pick the next task, and it must never run under RT-Thread: if it does,
 *    the FreeRTOS scheduler is being entered from somewhere and the image is
 *    already in a state nobody designed for.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "ft_core.h"

/* The stack unit constant in ft_core.h is derived from this port's
 * StackType_t. If a future port makes it a 32-bit type, every task stack in the
 * system would silently be a quarter of what the caller asked for, so fail the
 * build instead. */
_Static_assert(sizeof(StackType_t) == FT_STACK_UNIT_BYTES,
               "ft_core.h FT_STACK_UNIT_BYTES no longer matches StackType_t");

/* Referenced by stack_macros.h, which some port code inlines even though it
 * never runs under RT-Thread. Nothing is scheduled by FreeRTOS here, so the
 * array only has to exist for the linker. */
void *pxCurrentTCBs[configNUM_CORES] = { NULL };

static BaseType_t ft_result(int rc)
{
    return (rc >= 0) ? pdTRUE : pdFALSE;
}

static void ft_warn_once(rt_uint32_t *flag, const char *what)
{
    if (*flag == 0)
    {
        *flag = 1;
        rt_kprintf("freertos-shim: %s\n", what);
    }
}

/* ------------------------------------------------------------------ queue -- */

QueueHandle_t xQueueGenericCreate(const UBaseType_t uxQueueLength,
                                  const UBaseType_t uxItemSize,
                                  const uint8_t ucQueueType)
{
    switch (ucQueueType)
    {
    case queueQUEUE_TYPE_MUTEX:
        return (QueueHandle_t)ft_ipc_create_mutex_ex(0);

    case queueQUEUE_TYPE_RECURSIVE_MUTEX:
        return (QueueHandle_t)ft_ipc_create_mutex_ex(1);

    case queueQUEUE_TYPE_COUNTING_SEMAPHORE:
        /* The initial count is applied by xQueueCreateCountingSemaphore. */
        return (QueueHandle_t)ft_ipc_create(uxQueueLength ? uxQueueLength : 1, 0);

    default:
        return (QueueHandle_t)ft_ipc_create(uxQueueLength, uxItemSize);
    }
}

QueueHandle_t xQueueCreateMutex(const uint8_t ucQueueType)
{
    return (QueueHandle_t)ft_ipc_create_mutex_ex(ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX);
}

QueueHandle_t xQueueCreateMutexStatic(const uint8_t ucQueueType,
                                      StaticQueue_t *pxStaticQueue)
{
    return (QueueHandle_t)ft_ipc_create_mutex_static(
        pxStaticQueue, ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX);
}

QueueHandle_t xQueueCreateCountingSemaphore(const UBaseType_t uxMaxCount,
                                             const UBaseType_t uxInitialCount)
{
    return (QueueHandle_t)ft_ipc_create_sem(uxMaxCount, uxInitialCount);
}

BaseType_t xQueueGenericSend(QueueHandle_t xQueue,
                             const void *const pvItemToQueue,
                             TickType_t xTicksToWait,
                             const BaseType_t xCopyPosition)
{
    RT_UNUSED(xCopyPosition);   /* both positions are "append" for us */
    return ft_result(ft_ipc_send((struct ft_ipc *)xQueue, pvItemToQueue, xTicksToWait));
}

BaseType_t xQueueGenericSendFromISR(QueueHandle_t xQueue,
                                    const void *const pvItemToQueue,
                                    BaseType_t *const pxHigherPriorityTaskWoken,
                                    const BaseType_t xCopyPosition)
{
    RT_UNUSED(xCopyPosition);
    return ft_result(ft_ipc_send_isr((struct ft_ipc *)xQueue, pvItemToQueue,
                                     (ft_base_t *)pxHigherPriorityTaskWoken));
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *const pvBuffer,
                         TickType_t xTicksToWait)
{
    return ft_result(ft_ipc_receive((struct ft_ipc *)xQueue, pvBuffer, xTicksToWait));
}

BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void *const pvBuffer,
                                BaseType_t *const pxHigherPriorityTaskWoken)
{
    return ft_result(ft_ipc_receive_isr((struct ft_ipc *)xQueue, pvBuffer,
                                        (ft_base_t *)pxHigherPriorityTaskWoken));
}

BaseType_t xQueueIsQueueEmptyFromISR(const QueueHandle_t xQueue)
{
    return ft_ipc_is_empty((struct ft_ipc *)xQueue);
}

BaseType_t xQueueSemaphoreTake(QueueHandle_t xQueue, TickType_t xTicksToWait)
{
    return ft_result(ft_ipc_take((struct ft_ipc *)xQueue, xTicksToWait));
}

BaseType_t xQueueGiveFromISR(QueueHandle_t xQueue,
                             BaseType_t *const pxHigherPriorityTaskWoken)
{
    return ft_result(ft_ipc_give_isr((struct ft_ipc *)xQueue,
                                     (ft_base_t *)pxHigherPriorityTaskWoken));
}

UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue)
{
    return (UBaseType_t)ft_ipc_messages_waiting((struct ft_ipc *)xQueue);
}

UBaseType_t uxQueueSpacesAvailable(const QueueHandle_t xQueue)
{
    return (UBaseType_t)ft_ipc_spaces_available((struct ft_ipc *)xQueue);
}

void vQueueDelete(QueueHandle_t xQueue)
{
    ft_ipc_delete((struct ft_ipc *)xQueue);
}

BaseType_t xQueueTakeMutexRecursive(QueueHandle_t xMutex, TickType_t xTicksToWait)
{
    return ft_result(ft_ipc_take((struct ft_ipc *)xMutex, xTicksToWait));
}

BaseType_t xQueueGiveMutexRecursive(QueueHandle_t xMutex)
{
    return ft_result(ft_ipc_give((struct ft_ipc *)xMutex));
}

TaskHandle_t xQueueGetMutexHolder(QueueHandle_t xSemaphore)
{
    return (TaskHandle_t)ft_ipc_mutex_holder((struct ft_ipc *)xSemaphore);
}

/* ------------------------------------------------------------------- task -- */

/* xTaskCreate() is a static inline in task.h that forwards here, so it must not
 * be defined again: the header already provides it in every translation unit. */
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t pxTaskCode,
                                   const char *const pcName,
                                   const configSTACK_DEPTH_TYPE usStackDepth,
                                   void *const pvParameters,
                                   UBaseType_t uxPriority,
                                   TaskHandle_t *const pxCreatedTask,
                                   const BaseType_t xCoreID)
{
    static rt_uint32_t warned;

    /* RT-Thread schedules on the PRO core; S31's second hart stays in reset
     * until the SMP work lands, so there is nowhere else to put a task. */
    if (xCoreID != tskNO_AFFINITY && xCoreID != 0)
    {
        ft_warn_once(&warned,
                     "xTaskCreatePinnedToCore(core != 0): pinned tasks run on "
                     "core 0 until SMP support lands");
    }

    void *thread = ft_task_create(pcName, pxTaskCode, pvParameters,
                                  (ft_ubase_t)usStackDepth, (ft_ubase_t)uxPriority,
                                  (void **)pxCreatedTask);

    return (thread != RT_NULL) ? pdPASS : pdFAIL;
}

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return (TaskHandle_t)ft_task_self();
}

TickType_t xTaskGetTickCount(void)
{
    return (TickType_t)ft_task_tick_count();
}

TickType_t xTaskGetTickCountFromISR(void)
{
    return (TickType_t)ft_task_tick_count_isr();
}

BaseType_t xTaskGetSchedulerState(void)
{
    return (BaseType_t)ft_task_scheduler_state();
}

void vTaskDelay(const TickType_t xTicksToDelay)
{
    ft_task_delay(xTicksToDelay);
}

void vTaskDelete(TaskHandle_t xTaskToDelete)
{
    ft_task_delete(xTaskToDelete);
}

void vTaskSuspend(TaskHandle_t xTaskToSuspend)
{
    ft_task_suspend(xTaskToSuspend);
}

void vTaskSuspendAll(void)
{
    ft_task_suspend_all();
}

BaseType_t xTaskResumeAll(void)
{
    return ft_task_resume_all();
}

void vTaskSwitchContext(void)
{
    /* Must never be reached: it would mean the FreeRTOS scheduler is running,
     * which this image does not do. Print loudly the first time. */
    static rt_uint32_t warned;

    ft_warn_once(&warned,
                 "vTaskSwitchContext() called - the FreeRTOS scheduler is not "
                 "running under RT-Thread");
}

void vTaskPrioritySet(TaskHandle_t xTask, UBaseType_t uxNewPriority)
{
    ft_task_set_priority(xTask, (ft_ubase_t)uxNewPriority);
}

UBaseType_t uxTaskPriorityGet(const TaskHandle_t xTask)
{
    return (UBaseType_t)ft_task_get_priority(xTask);
}

void vTaskSetThreadLocalStoragePointer(TaskHandle_t xTaskToSet,
                                       BaseType_t xIndex,
                                       void *pvValue)
{
    ft_task_set_tls(xTaskToSet, (ft_ubase_t)xIndex, pvValue);
}

void *pvTaskGetThreadLocalStoragePointer(TaskHandle_t xTaskToQuery,
                                         BaseType_t xIndex)
{
    return ft_task_get_tls(xTaskToQuery, (ft_ubase_t)xIndex);
}

/* ------------------------------------------------------------ event group -- */

EventGroupHandle_t xEventGroupCreate(void)
{
    return (EventGroupHandle_t)ft_event_create();
}

EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup,
                               const EventBits_t uxBitsToSet)
{
    return (EventBits_t)ft_event_set_bits((struct ft_event *)xEventGroup,
                                          (rt_uint32_t)uxBitsToSet);
}

EventBits_t xEventGroupClearBits(EventGroupHandle_t xEventGroup,
                                 const EventBits_t uxBitsToClear)
{
    return (EventBits_t)ft_event_clear_bits((struct ft_event *)xEventGroup,
                                            (rt_uint32_t)uxBitsToClear);
}

EventBits_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup,
                                const EventBits_t uxBitsToWaitFor,
                                const BaseType_t xClearOnExit,
                                const BaseType_t xWaitForAllBits,
                                TickType_t xTicksToWait)
{
    return (EventBits_t)ft_event_wait_bits((struct ft_event *)xEventGroup,
                                           (rt_uint32_t)uxBitsToWaitFor,
                                           xClearOnExit ? 1 : 0,
                                           xWaitForAllBits ? 1 : 0,
                                           xTicksToWait);
}

void vEventGroupDelete(EventGroupHandle_t xEventGroup)
{
    ft_event_delete((struct ft_event *)xEventGroup);
}

/* ------------------------------------------------------- static creation -- */

/*
 * xQueueCreateWithCaps() in idf_additions.c allocates a StaticQueue_t and calls
 * these three, so they are on esp_libc's and esp_driver_uart's init path even
 * when the caller never asked for static allocation.
 */

QueueHandle_t xQueueGenericCreateStatic(const UBaseType_t uxQueueLength,
                                        const UBaseType_t uxItemSize,
                                        uint8_t *pucQueueStorage,
                                        StaticQueue_t *pxStaticQueue,
                                        const uint8_t ucQueueType)
{
    switch (ucQueueType)
    {
    case queueQUEUE_TYPE_MUTEX:
        return (QueueHandle_t)ft_ipc_create_mutex_static(pxStaticQueue, 0);

    case queueQUEUE_TYPE_RECURSIVE_MUTEX:
        return (QueueHandle_t)ft_ipc_create_mutex_static(pxStaticQueue, 1);

    default:
        return (QueueHandle_t)ft_ipc_create_static_ram(uxQueueLength ? uxQueueLength : 1,
                                                       uxItemSize, pucQueueStorage,
                                                       pxStaticQueue);
    }
}

BaseType_t xQueueGenericGetStaticBuffers(QueueHandle_t xQueue,
                                         uint8_t **ppucQueueStorage,
                                         StaticQueue_t **ppxStaticQueue)
{
    if (xQueue == NULL || ppucQueueStorage == NULL || ppxStaticQueue == NULL)
    {
        return pdFALSE;
    }

    *ppucQueueStorage = (uint8_t *)ft_ipc_storage((struct ft_ipc *)xQueue);
    *ppxStaticQueue   = (StaticQueue_t *)ft_ipc_static_cb((struct ft_ipc *)xQueue);

    return pdTRUE;
}

QueueHandle_t xQueueCreateCountingSemaphoreStatic(const UBaseType_t uxMaxCount,
                                                  const UBaseType_t uxInitialCount,
                                                  StaticQueue_t *pxStaticQueue)
{
    struct ft_ipc *ipc = ft_ipc_create_static_ram(uxMaxCount ? uxMaxCount : 1, 0,
                                                  NULL, pxStaticQueue);
    UBaseType_t i;

    if (ipc == NULL)
    {
        return NULL;
    }

    for (i = 0; i < uxInitialCount; i++)
    {
        if (ft_ipc_give(ipc) != FT_OK)
        {
            break;
        }
    }

    return (QueueHandle_t)ipc;
}

/* ------------------------------------------------------- notifications -- */

/*
 * Only the generic functions exist as symbols. task.h defines xTaskNotify,
 * xTaskNotifyWait, ulTaskNotifyTake and friends as function-like macros over
 * these, so defining the short names here is a redefinition error - the same
 * trap as xTaskCreate being a static inline in task.h. The compiler caught it,
 * which is why this block talks to the generic layer only.
 *
 * configTASK_NOTIFICATION_ARRAY_ENTRIES is 1 in this configuration, so index 0
 * is the only valid index and the others are ignored.
 */

BaseType_t xTaskGenericNotify(TaskHandle_t xTaskToNotify,
                              UBaseType_t uxIndexToNotify,
                              uint32_t ulValue,
                              eNotifyAction eAction,
                              uint32_t *pulPreviousNotificationValue)
{
    RT_UNUSED(uxIndexToNotify);

    if (pulPreviousNotificationValue != NULL)
    {
        *pulPreviousNotificationValue = ft_task_notify_peek(xTaskToNotify);
    }

    return ft_result(ft_task_notify(xTaskToNotify, ulValue, (ft_base_t)eAction));
}

BaseType_t xTaskGenericNotifyFromISR(TaskHandle_t xTaskToNotify,
                                     UBaseType_t uxIndexToNotify,
                                     uint32_t ulValue,
                                     eNotifyAction eAction,
                                     uint32_t *pulPreviousNotificationValue,
                                     BaseType_t *pxHigherPriorityTaskWoken)
{
    RT_UNUSED(uxIndexToNotify);

    if (pulPreviousNotificationValue != NULL)
    {
        *pulPreviousNotificationValue = ft_task_notify_peek(xTaskToNotify);
    }

    return ft_result(ft_task_notify_isr(xTaskToNotify, ulValue, (ft_base_t)eAction,
                                        (ft_base_t *)pxHigherPriorityTaskWoken));
}

void vTaskGenericNotifyGiveFromISR(TaskHandle_t xTaskToNotify,
                                   UBaseType_t uxIndexToNotify,
                                   BaseType_t *pxHigherPriorityTaskWoken)
{
    RT_UNUSED(uxIndexToNotify);

    (void)ft_task_notify_isr(xTaskToNotify, 0, FT_NOTIFY_INCREMENT,
                             (ft_base_t *)pxHigherPriorityTaskWoken);
}

BaseType_t xTaskGenericNotifyWait(UBaseType_t uxIndexToWaitOn,
                                  uint32_t ulBitsToClearOnEntry,
                                  uint32_t ulBitsToClearOnExit,
                                  uint32_t *pulNotificationValue,
                                  TickType_t xTicksToWait)
{
    ft_base_t timed_out = 0;
    rt_uint32_t value;

    RT_UNUSED(uxIndexToWaitOn);

    value = ft_task_notify_wait(ulBitsToClearOnEntry, ulBitsToClearOnExit,
                                xTicksToWait, &timed_out);

    if (pulNotificationValue != NULL)
    {
        *pulNotificationValue = value;
    }

    return timed_out ? pdFALSE : pdTRUE;
}

uint32_t ulTaskGenericNotifyTake(UBaseType_t uxIndexToWaitOn,
                                 BaseType_t xClearCountOnExit,
                                 TickType_t xTicksToWait)
{
    RT_UNUSED(uxIndexToWaitOn);

    return ft_task_notify_take(xClearCountOnExit, xTicksToWait);
}

BaseType_t xTaskGenericNotifyStateClear(TaskHandle_t xTask,
                                        UBaseType_t uxIndexToClear)
{
    RT_UNUSED(uxIndexToClear);

    /* FreeRTOS returns pdTRUE when a notification was pending and clears it. */
    return ft_task_notify_clear(xTask) ? pdTRUE : pdFALSE;
}

/* ------------------------------------------------------------ timeouts -- */

/* TimeOut_t is { BaseType_t xOverflowCount; TickType_t xTimeOnEntering; } and
 * ft_timeout_t mirrors it, so the cast below is a reinterpretation of the same
 * two fields. */
_Static_assert(sizeof(TimeOut_t) == sizeof(ft_timeout_t),
               "ft_timeout_t must match FreeRTOS TimeOut_t field for field");

void vTaskSetTimeOutState(TimeOut_t *const pxTimeOut)
{
    ft_timeout_set((ft_timeout_t *)pxTimeOut);
}

void vTaskInternalSetTimeOutState(TimeOut_t *const pxTimeOut)
{
    /* Same bookkeeping; FreeRTOS splits it only to skip a scheduler-state check
     * that this port does not need. */
    ft_timeout_set((ft_timeout_t *)pxTimeOut);
}

BaseType_t xTaskCheckForTimeOut(TimeOut_t *const pxTimeOut,
                                TickType_t *const pxTicksToWait)
{
    /* TickType_t is unsigned int and ft_tick_t is unsigned long: same width on
     * rv32, different types, so the remaining-tick value has to be copied
     * rather than aliased. */
    ft_tick_t remaining = *pxTicksToWait;
    ft_base_t expired = ft_timeout_check((ft_timeout_t *)pxTimeOut, &remaining);

    *pxTicksToWait = remaining;

    return expired ? pdTRUE : pdFALSE;
}
