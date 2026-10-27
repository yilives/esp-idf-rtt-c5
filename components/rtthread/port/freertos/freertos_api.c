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
 * 1. Static allocation falls back to the heap. ft_ipc is about 170 bytes and
 *    StaticQueue_t is about 120, so a caller's control block can never hold the
 *    object. Silently overflowing it would corrupt whatever follows; failing
 *    would break esp_libc's lock initialisation. Allocating instead wastes the
 *    caller's buffer (a few hundred bytes, once) and keeps the system running.
 *    A message is printed the first time so the cost is visible.
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
    static rt_uint32_t warned;

    RT_UNUSED(pxStaticQueue);
    ft_warn_once(&warned,
                 "static mutex control blocks are too small for ft_ipc, "
                 "falling back to the heap");

    return (QueueHandle_t)ft_ipc_create_mutex_ex(ucQueueType == queueQUEUE_TYPE_RECURSIVE_MUTEX);
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
