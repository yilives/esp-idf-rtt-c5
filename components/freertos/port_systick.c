/*
 * SPDX-FileCopyrightText: 2017-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include <stdint.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#if CONFIG_IDF_RTOS_RTTHREAD
#include <rtthread.h>
#include <rthw.h>

_Static_assert(CONFIG_FREERTOS_HZ == RT_TICK_PER_SECOND,
               "IDF SYSTIMER and RT-Thread tick rates must match");
#endif
#if ( !CONFIG_FREERTOS_SMP && ( configNUM_CORES > 1 ) )
/* Required for xTaskIncrementTickOtherCores() */
#include "esp_private/freertos_idf_additions_priv.h"
#endif /* ( !CONFIG_FREERTOS_SMP && ( configNUM_CORES > 1 ) ) */

#if CONFIG_FREERTOS_SYSTICK_USES_CCOUNT
#if CONFIG_FREERTOS_CORETIMER_0
#define SYSTICK_INTR_ID     (ETS_INTERNAL_TIMER0_INTR_SOURCE + ETS_INTERNAL_INTR_SOURCE_OFF)
#else /* CONFIG_FREERTOS_CORETIMER_1 */
#define SYSTICK_INTR_ID     (ETS_INTERNAL_TIMER1_INTR_SOURCE + ETS_INTERNAL_INTR_SOURCE_OFF)
#endif
#else /* CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER */
#define SYSTICK_INTR_ID     (ETS_SYSTIMER_TARGET0_INTR_SOURCE)
#endif /* CONFIG_FREERTOS_SYSTICK_USES_CCOUNT */

BaseType_t xPortSysTickHandler(void);

/* --------------------------------------------- SYSTIMER Implementation -----------------------------------------------
 * Implementation of a tick interrupt using the SYSTIMER perpiheral
 * Todo: Abstract setup and operation in a separate SYSTIMER driver (IDF-6976)
 * ------------------------------------------------------------------------------------------------------------------ */

#if CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER

#include "soc/periph_defs.h"
#include "hal/systimer_hal.h"
#include "hal/systimer_ll.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_private/systimer.h"
#include "esp_private/periph_ctrl.h"
#ifdef CONFIG_PM_TRACE
#include "esp_private/pm_trace.h"
#endif //CONFIG_PM_TRACE

_Static_assert(SOC_CPU_CORES_NUM <= SYSTIMER_LL_ALARM_NUM - 1, "the number of cores must match the number of core alarms in SYSTIMER");

void SysTickIsrHandler(void *arg);

static uint32_t s_handled_systicks[configNUM_CORES] = { 0 };

/**
 * @brief Set up the systimer peripheral to generate the tick interrupt
 *
 * Both timer alarms are configured in periodic mode.
 * It is done at the same time so SysTicks for both CPUs occur at the same time or very close.
 * Shifts a time of triggering interrupts for core 0 and core 1.
 */
void vSystimerSetup(void)
{
    unsigned cpuid = xPortGetCoreID();
#ifdef CONFIG_FREERTOS_CORETIMER_SYSTIMER_LVL3
    const unsigned level = ESP_INTR_FLAG_LEVEL3;
#else
    const unsigned level = ESP_INTR_FLAG_LEVEL1;
#endif
    /* Systimer HAL layer object */
    static systimer_hal_context_t systimer_hal;
    /* set system timer interrupt vector */

    unsigned flags = ESP_INTR_FLAG_IRAM | level;
#if CONFIG_IDF_RTOS_RTTHREAD
    /* The real RT-Thread tick/scheduler/timer code lives in flash. Mask the
     * interrupt while cache is disabled; the ISR catches up elapsed ticks.
     */
    flags = level;
#endif
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_SYSTIMER_TARGET0_INTR_SOURCE + cpuid, flags, SysTickIsrHandler, &systimer_hal, NULL));

    if (cpuid == 0) {
        PERIPH_RCC_ACQUIRE_ATOMIC(PERIPH_SYSTIMER_MODULE, ref_count) {
            if (ref_count == 0) {
                systimer_ll_enable_bus_clock(true);
                systimer_ll_enable_sys_clock(true);
                systimer_ll_reset_register();
            }
        }
        systimer_hal_init(&systimer_hal);
        systimer_hal_tick_rate_ops_t ops = {
            .ticks_to_us = systimer_ticks_to_us,
            .us_to_ticks = systimer_us_to_ticks,
        };
        systimer_hal_set_tick_rate_ops(&systimer_hal, &ops);
        systimer_ll_set_counter_value(systimer_hal.dev, SYSTIMER_COUNTER_OS_TICK, 0);
        systimer_ll_apply_counter_value(systimer_hal.dev, SYSTIMER_COUNTER_OS_TICK);

        for (cpuid = 0; cpuid < SOC_CPU_CORES_NUM; cpuid++) {
            // Set stall option and alarm mode to default state. Below they will be set to a required state.
            systimer_hal_counter_can_stall_by_cpu(&systimer_hal, SYSTIMER_COUNTER_OS_TICK, cpuid, false);
            uint32_t alarm_id = SYSTIMER_ALARM_OS_TICK_CORE0 + cpuid;
            systimer_hal_select_alarm_mode(&systimer_hal, alarm_id, SYSTIMER_ALARM_MODE_ONESHOT);
        }

        for (cpuid = 0; cpuid < configNUM_CORES; ++cpuid) {
            uint32_t alarm_id = SYSTIMER_ALARM_OS_TICK_CORE0 + cpuid;

            /* configure the timer */
            systimer_hal_connect_alarm_counter(&systimer_hal, alarm_id, SYSTIMER_COUNTER_OS_TICK);
            systimer_hal_set_alarm_period(&systimer_hal, alarm_id, 1000000UL / CONFIG_FREERTOS_HZ);
            systimer_hal_select_alarm_mode(&systimer_hal, alarm_id, SYSTIMER_ALARM_MODE_PERIOD);
            systimer_hal_counter_can_stall_by_cpu(&systimer_hal, SYSTIMER_COUNTER_OS_TICK, cpuid, true);
            if (cpuid == 0) {
                systimer_hal_enable_alarm_int(&systimer_hal, alarm_id);
                systimer_hal_enable_counter(&systimer_hal, SYSTIMER_COUNTER_OS_TICK);
#ifndef CONFIG_FREERTOS_UNICORE
                // SysTick of core 0 and core 1 are shifted by half of period
                systimer_hal_counter_value_advance(&systimer_hal, SYSTIMER_COUNTER_OS_TICK, 1000000UL / CONFIG_FREERTOS_HZ / 2);
#endif
            }
        }
    } else {
        uint32_t alarm_id = SYSTIMER_ALARM_OS_TICK_CORE0 + cpuid;
        systimer_hal_enable_alarm_int(&systimer_hal, alarm_id);
    }
}

/**
 * @brief Systimer interrupt handler.
 *
 * The Systimer interrupt for SysTick works in periodic mode no need to calc the next alarm.
 * If a timer interrupt is ever serviced more than one tick late, it is necessary to process multiple ticks.
 */
void SysTickIsrHandler(void *arg)
{
    uint32_t cpuid = xPortGetCoreID();
    systimer_hal_context_t *systimer_hal = (systimer_hal_context_t *)arg;
#ifdef CONFIG_PM_TRACE
    ESP_PM_TRACE_ENTER(TICK, cpuid);
#endif

    uint32_t alarm_id = SYSTIMER_ALARM_OS_TICK_CORE0 + cpuid;
    do {
        systimer_ll_clear_alarm_int(systimer_hal->dev, alarm_id);

        uint32_t diff = systimer_hal_get_counter_value(systimer_hal, SYSTIMER_COUNTER_OS_TICK) / systimer_ll_get_alarm_period(systimer_hal->dev, alarm_id) - s_handled_systicks[cpuid];
        if (diff > 0) {
            if (s_handled_systicks[cpuid] == 0) {
                s_handled_systicks[cpuid] = diff;
                diff = 1;
            } else {
                s_handled_systicks[cpuid] += diff;
            }

            do {
                xPortSysTickHandler();
            } while (--diff);
        }
    } while (systimer_ll_is_alarm_int_fired(systimer_hal->dev, alarm_id));

#ifdef CONFIG_PM_TRACE
    ESP_PM_TRACE_EXIT(TICK, cpuid);
#endif
}
#endif /* CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER */

/* ------------------------------------------------ Common Port Tick ---------------------------------------------------
 * Tick related functions common across all ports
 * ------------------------------------------------------------------------------------------------------------------ */

/**
 * @brief Initialize the tick interrupt timer
 *
 * - CCOUNT timer is used if CONFIG_FREERTOS_SYSTICK_USES_CCOUNT is set
 * - SYSTIMER is used if CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER is set
 */
void vPortSetupTimer(void)
{
#if CONFIG_FREERTOS_SYSTICK_USES_CCOUNT
    extern void _frxt_tick_timer_init(void);
    extern void _xt_tick_divisor_init(void);
    /* Init the tick divisor value */
    _xt_tick_divisor_init();
    _frxt_tick_timer_init();
#else /* CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER */
    vSystimerSetup();
#endif /* CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER */
}

/**
 * @brief Handler of SysTick
 *
 * The function is called from:
 *  - _frxt_timer_int for xtensa with CONFIG_FREERTOS_SYSTICK_USES_CCOUNT
 *  - SysTickIsrHandler for xtensa with CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER
 *  - SysTickIsrHandler for riscv
 */
BaseType_t xPortSysTickHandler(void)
{
#if configBENCHMARK
    portbenchmarkIntLatency();
#endif //configBENCHMARK
    traceISR_ENTER(SYSTICK_INTR_ID);

    // Call IDF Tick Hook
    extern void esp_vApplicationTickHook(void);
    esp_vApplicationTickHook();

#if CONFIG_IDF_RTOS_RTTHREAD
    /* vectors_rtthread.S brackets C3/C6 dispatch with rt_interrupt_enter/leave.
     * Protect the kernel tick from nesting; requested thread switches run at
     * the outermost trap exit.
     */
    /*
     * LOCAL mask, not rt_hw_interrupt_disable().
     *
     * Under RT_USING_SMP rthw.h:146 defines rt_hw_interrupt_disable as
     * rt_cpus_lock - which takes the GLOBAL _cpus_lock spinlock and bumps
     * current_thread->cpus_lock_nest. Doing that from an interrupt handler is
     * wrong twice over: the handler runs on the ISR stack with the preempted
     * thread's window state spilled elsewhere, and the spinlock's S32C1I plus the
     * extra windowed-ABI calls are exactly what the crash log shows
     * (rt_cpus_lock -> rt_soft_atomic_load -> rt_hw_interrupt_disable with
     * sp = 0x3fc80020, below DRAM).
     *
     * rt_hw_local_irq_disable only raises PS.INTLEVEL on this core - which is all
     * a handler needs, since it is already running with interrupts masked at its
     * own level.
     */


    /*
     * Acknowledge and park while the other core has its instruction cache off.
     *
     * Everything below this point up to the acknowledgement is IRAM-resident, so
     * it is safe to run with the cache disabled. The park loop is what CPU0's
     * spi_flash_other_core_stop() waits for; without it that wait times out and
     * CPU0 proceeds to touch flash while this core is still fetching from it.
     *
     * Guarded on the flag first so the common tick costs one load.
     */
#if defined(__XTENSA__) && (configNUM_CORES > 1)
    /* Dual-core Xtensa only: a cache-off handshake between CPU0 and CPU1, plus a
     * debugging probe for it. `xt_rt_flash_op`, `xt_rt_flash_ack` and `rt_xt_pk`
     * are all defined in the Xtensa port; a single-core RISC-V build (the C5) has
     * none of them and the link fails with undefined references to all three.
     *
     * Guarded, not deleted - this is active work on the SMP Xtensa target, and it
     * is correct there.
     *
     * This is the THIRD distinct block of Xtensa-only instrumentation to appear in
     * this one file during this port (after rt_xt_br and rt_xt_sst). Anything added
     * to xPortSysTickHandler needs to answer "does this exist on RISC-V?" before it
     * compiles here. */
    {
        extern volatile rt_uint32_t xt_rt_flash_op;
        extern volatile rt_uint32_t xt_rt_flash_ack;

        if (xt_rt_flash_op != 0)
        {
            xt_rt_flash_ack = 1;

            while (xt_rt_flash_op != 0)
            {
                /* CPU0 clears the flag once its cache is back on. */
            }
        }
    }
#endif /* __XTENSA__ && configNUM_CORES > 1 */



    {
        extern volatile unsigned int rt_xt_ip3[8];
        extern int esp_rom_printf(const char *fmt, ...);
        static volatile int __ip3_n;
        if ((int)xPortGetCoreID() == 1 && rt_xt_ip3[0] > 0 && __ip3_n < 3)
        {
            __ip3_n++;
            {
                extern volatile unsigned int rt_xt_nf[8];
                esp_rom_printf("\nNF n=%u t=%08x pre=%02x rel=%u post=%02x\n",
                               rt_xt_nf[0], rt_xt_nf[1], rt_xt_nf[2],
                               rt_xt_nf[3], rt_xt_nf[4]);
            }
        }
    }

    rt_base_t level = rt_hw_local_irq_disable();
    rt_tick_increase();
    rt_hw_local_irq_enable(level);
    traceISR_EXIT();
    return pdFALSE;
#else
    // Call FreeRTOS Increment tick function
    BaseType_t xSwitchRequired;
#if CONFIG_FREERTOS_SMP
    UBaseType_t uxSavedStatus = taskENTER_CRITICAL_FROM_ISR();
    // Amazon SMP FreeRTOS requires that only core 0 calls xTaskIncrementTick()
#if ( configNUM_CORES > 1 )
    if (portGET_CORE_ID() == 0) {
        xSwitchRequired = xTaskIncrementTick();
    } else {
        xSwitchRequired = pdFALSE;
    }
#else /* configNUM_CORES > 1 */
    xSwitchRequired = xTaskIncrementTick();
#endif /* configNUM_CORES > 1 */
    taskEXIT_CRITICAL_FROM_ISR(uxSavedStatus);
#else /* !CONFIG_FREERTOS_SMP */
#if ( configNUM_CORES > 1 )
    /*
    Multi-core IDF FreeRTOS requires that...
        - core 0 calls xTaskIncrementTick()
        - core 1 calls xTaskIncrementTickOtherCores()
    */
    if (xPortGetCoreID() == 0) {
        xSwitchRequired = xTaskIncrementTick();
    } else {
        xSwitchRequired = xTaskIncrementTickOtherCores();
    }
#else /* configNUM_CORES > 1 */
    /*
    Vanilla (single core) FreeRTOS expects that xTaskIncrementTick() cannot be interrupted (i.e., no nested
    interrupts). Thus we have to disable interrupts before calling it.
    */
    UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    xSwitchRequired = xTaskIncrementTick();
    portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
#endif /* configNUM_CORES > 1 */
#endif /* !CONFIG_FREERTOS_SMP */

    // Check if yield is required
    if (xSwitchRequired != pdFALSE) {
        portYIELD_FROM_ISR();
    } else {
        traceISR_EXIT();
    }
    return xSwitchRequired;
#endif
}
