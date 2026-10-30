/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include "esp_private/esp_system_attr.h"
#include "esp_err.h"
#include "esp_compiler.h"
#include "esp_macros.h"

#include "esp_system.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "soc/soc_caps.h"
#include "esp_cpu.h"

#if CONFIG_IDF_RTOS_RTTHREAD
#include "rtthread.h"
#include "rthw.h"

#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
#ifndef RT_USING_SMP
#error "RT-Thread requires RT_USING_SMP when IDF starts multiple cores."
#elif RT_CPUS_NR != SOC_CPU_CORES_NUM
#error "RT-Thread and IDF must use the same number of cores."
#endif
#endif
#endif

#include "esp_private/startup_internal.h"

// Ensure that system configuration matches the underlying number of cores.
// This should enable us to avoid checking for both every time.
#if !(SOC_CPU_CORES_NUM > 1) && !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
#error "System has been configured to run on multiple cores, but target SoC only has a single core."
#endif

uint64_t g_startup_time = 0;

// App entry point for core 0
extern void esp_startup_start_app(void);

// Entry point for core 0 from hardware init (port layer)
#if defined(__APPLE__) && defined(__MACH__)
static void start_cpu0_default(void);
__attribute__((weak, noreturn)) void start_cpu0(void)
{
    start_cpu0_default();
    __builtin_unreachable();
}
#else
void start_cpu0(void) __attribute__((weak, alias("start_cpu0_default"))) __attribute__((noreturn));
#endif

#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
// Entry point for core [1..X] from hardware init (port layer)
#if defined(__APPLE__) && defined(__MACH__)
static void start_cpu_other_cores_default(void);
__attribute__((weak, noreturn)) void start_cpu_other_cores(void)
{
    start_cpu_other_cores_default();
    __builtin_unreachable();
}
#else
void start_cpu_other_cores(void) __attribute__((weak, alias("start_cpu_other_cores_default"))) __attribute__((noreturn));
#endif

// App entry point for core [1..X]
#if defined(__APPLE__) && defined(__MACH__)
static void esp_startup_start_app_other_cores_default(void);
__attribute__((weak, noreturn)) void esp_startup_start_app_other_cores(void)
{
    esp_startup_start_app_other_cores_default();
    __builtin_unreachable();
}
#else
void esp_startup_start_app_other_cores(void) __attribute__((weak, alias("esp_startup_start_app_other_cores_default"))) __attribute__((noreturn));
#endif

static volatile bool s_system_inited[SOC_CPU_CORES_NUM] = { false };

const sys_startup_fn_t g_startup_fn[SOC_CPU_CORES_NUM] = { [0] = start_cpu0,
#if SOC_CPU_CORES_NUM > 1
                                                           [1 ... SOC_CPU_CORES_NUM - 1] = start_cpu_other_cores
#endif
                                                         };

static volatile bool s_system_full_inited = false;
#else
const sys_startup_fn_t g_startup_fn[1] = { start_cpu0 };
#endif

ESP_LOG_ATTR_TAG(TAG, "cpu_start");

#if CONFIG_COMPILER_CXX_EXCEPTIONS && !CONFIG_IDF_TARGET_LINUX
/**
 * @brief A helper function for __do_global_ctors, which is in crtend.o.
 * It has been adapted from GCC source code. In GCC, it resides under
 * the USE_EH_FRAME_REGISTRY macro, which is not enabled in Espressif
 * toolchains to save small memory amount. Nevertheless, when C++ exceptions
 * are enabled this initialization becomes necessary.
 */
static void __do_global_ctors_1(void)
{
    struct object {
        long placeholder[10];
    };
    void __register_frame_info(const void *begin, struct object * ob);
    extern char __eh_frame[];

    static struct object ob;
    __register_frame_info(__eh_frame, &ob);
}
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS && !CONFIG_IDF_TARGET_LINUX

/* declare the start and stop symbols surrounding the array of init functions
 * registered by calling the system init function macros */
_SECTION_ATTR_SYMBOL_DECL_GENERIC(esp_system_init_fn_t, esp_sys_init_fn)

/**
 * @brief Call component init functions defined using the system init function macros.
 * The esp_system_init_fn_t structures describing these functions are collected into
 * an array [_esp_sys_init_fn_start, _esp_sys_init_fn_end) by the
 * linker. The functions are sorted by their priority value.
 * The sequence of the init function calls (sorted by priority) is documented in
 * system_init_fn.txt file.
 * @param stage_num Stage number of the init function call (0, 1).
 */
__attribute__((no_sanitize_undefined)) /* TODO: IDF-8133 */
static void do_system_init_fn(uint32_t stage_num)
{
    const esp_system_init_fn_t *p;

    int core_id = esp_cpu_get_core_id();
    for (p = _SECTION_START(esp_sys_init_fn); p < _SECTION_END(esp_sys_init_fn); ++p) {
        if (p->stage == stage_num && (p->cores & BIT(core_id)) != 0) {
            // During core init, stdout is not initialized yet, so use early logging.
            ESP_EARLY_LOGD(TAG, "calling init function: %p on core: %d", p->fn, core_id);
            esp_err_t err = (*(p->fn))();
            if (err != ESP_OK) {
                ESP_EARLY_LOGE(TAG, "init function %p has failed (0x%x), aborting", p->fn, err);
                abort();
            }
        }
    }

#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    s_system_inited[core_id] = true;
#endif
}

#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
static void  esp_startup_start_app_other_cores_default(void)
{
    while (1) {
        esp_rom_delay_us(UINT32_MAX);
    }
}

/* This function has to be in IRAM, as while it is running on CPU1, CPU0 may do some flash operations
 * (e.g. initialize the core dump), which means that cache will be disabled.
 */
static void ESP_SYSTEM_IRAM_ATTR start_cpu_other_cores_default(void)
{
    do_system_init_fn(ESP_SYSTEM_INIT_STAGE_SECONDARY);

#if !CONFIG_IDF_RTOS_RTTHREAD
    while (!s_system_full_inited) {
        esp_rom_delay_us(100);
    }

    esp_startup_start_app_other_cores();
#else
    // The primary core publishes this flag only after kernel initialization.
    while (!__atomic_load_n(&s_system_full_inited, __ATOMIC_ACQUIRE)) {
        esp_rom_delay_us(100);
    }
#ifdef RT_USING_SMP
    rt_secondary_cpu_entry();
#endif
    ESP_EARLY_LOGE(TAG, "RT-Thread secondary CPU entry returned");
    abort();
#endif
}
#endif

static void do_core_init(void)
{
#if CONFIG_IDF_RTOS_RTTHREAD && defined(RT_USING_HEAP)
    /* The RT-Thread heap has to exist before any CORE-stage init function runs,
     * not after. One of them allocates.
     *
     * esp_libc_init -> esp_libc_locks_init creates per-file mutexes, and on the
     * FreeRTOS shim a mutex is an rt_malloc'ed object, so the very first
     * rt_malloc of the boot happens inside do_system_init_fn(). With the heap
     * still uninitialised, rt_smem_alloc() dereferences system_heap == NULL at
     * src/mem.c:287 and the CPU takes a load access fault at address 0xc
     * (offsetof(struct rt_object, type)).
     *
     * The observed panic, for the record: MEPC 0x400112ce (rt_object_get_type),
     * A0 = 0, MTVAL = 0xc, called from xQueueCreateMutexStatic via
     * esp_libc_locks_init. Moving this up is the whole fix - nothing else about
     * the heap changed.
     *
     * Ordering with respect to IDF's own heap: heap_caps_init has already run
     * as an earlier CORE-stage function, and __heap_start__/__heap_end__ are
     * linker symbols describing a region IDF deliberately leaves out of its own
     * allocator, so initialising RT-Thread's heap here cannot take memory from
     * it. */
    extern int __heap_start__;
    extern int __heap_end__;
    rt_system_heap_init((void *)&__heap_start__, (void *)&__heap_end__);
#endif // CONFIG_IDF_RTOS_RTTHREAD && defined(RT_USING_HEAP)

    do_system_init_fn(ESP_SYSTEM_INIT_STAGE_CORE);
}

static void do_secondary_init(void)
{
#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    // The port layer transferred control to this function with other cores 'paused',
    // resume execution so that cores might execute component initialization functions.
    startup_resume_other_cores();
#endif

    // Execute initialization functions esp_system_init_fn_t assigned to the main core. While
    // this is happening, all other cores are executing the initialization functions
    // assigned to them since they have been resumed already.
    do_system_init_fn(ESP_SYSTEM_INIT_STAGE_SECONDARY);

#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    // Wait for all cores to finish secondary init.
    volatile bool system_inited = false;

    while (!system_inited) {
        system_inited = true;
        for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
            system_inited &= s_system_inited[i];
        }
        esp_rom_delay_us(100);
    }
#endif
}

static void start_cpu0_default(void)
{
    // Initialize core components and services.
    // Operations that needs the cache to be disabled have to be done here.
    do_core_init();

#if !CONFIG_IDF_TARGET_LINUX
    // Execute constructors.
#if CONFIG_COMPILER_CXX_EXCEPTIONS
    __do_global_ctors_1();
#endif
    extern void __libc_init_array(void);
    __libc_init_array();
#endif // !CONFIG_IDF_TARGET_LINUX

    /* ----------------------------------Separator-----------------------------
     * After this stage, other CPU start running with the cache, however the scheduler (and ipc service) is not available.
     * Don't touch the cache/MMU until the OS is up.
     */

    // Execute init functions of other components; blocks
    // until all cores finish (when !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE).
    do_secondary_init();

#if !CONFIG_IDF_RTOS_RTTHREAD
#if SOC_CPU_CORES_NUM > 1 && !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    s_system_full_inited = true;
#endif

    esp_startup_start_app();
#else
#if defined(__GNUC__) && defined(RT_USING_USER_MAIN) && CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    // Public RT-Thread entry: board init, main thread and scheduler startup.
    // Board init reuses the heap initialized once by do_core_init() on C3/C6.
    entry();
#else
    // Follow src/components.c using public interfaces. IDF has already brought
    // up the other cores, so do not invoke rt_hw_secondary_cpu_up() again.
    // This path needs board/tick setup and application thread creation from
    // the port; rt_application_init() is not declared in the public headers.
    rt_hw_local_irq_disable();
#ifdef RT_USING_SMP
    rt_hw_spin_lock_init(&_cpus_lock);
    rt_hw_spin_lock(&_cpus_lock);
#endif
    rt_system_timer_init();
    rt_system_scheduler_init();
#ifdef RT_USING_SIGNALS
    rt_system_signal_init();
#endif
    rt_system_timer_thread_init();
    rt_thread_idle_init();
    rt_thread_defunct_init();
#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    // Release secondary cores only once scheduler data and idle threads exist.
    // The RT-Thread secondary entry must take _cpus_lock before scheduling.
    __atomic_store_n(&s_system_full_inited, true, __ATOMIC_RELEASE);
#endif
    rt_system_scheduler_start();
#endif
    ESP_EARLY_LOGE(TAG, "RT-Thread scheduler startup returned");
    abort();
#endif

    ESP_INFINITE_LOOP();
}

#if CONFIG_IDF_TARGET_LINUX && !defined(ESP_SYSTEM_LINUX_NO_MAIN)
__attribute__((weak)) int main(int argc, char **argv)
{
    start_cpu0();
    return 0;
}
#endif // CONFIG_IDF_TARGET_LINUX
