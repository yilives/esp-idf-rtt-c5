# IDF discovers dependencies before Kconfig is available. Do not resolve external
# sources during that pass; the normal pass registers nothing when disabled.
idf_build_get_property(target IDF_TARGET)
idf_build_get_property(non_os_build NON_OS_BUILD)
if(non_os_build)
    idf_component_register()
    return()
endif()
if(CMAKE_BUILD_EARLY_EXPANSION)
    if(target STREQUAL "esp32c3" OR target STREQUAL "esp32c6" OR target STREQUAL "esp32s31")
        idf_component_register(PRIV_REQUIRES freertos riscv)
    else()
        idf_component_register()
    endif()
    return()
endif()
if(NOT CONFIG_IDF_RTOS_RTTHREAD)
    idf_component_register()
    return()
endif()
if(NOT (target STREQUAL "esp32c3" OR target STREQUAL "esp32c6" OR target STREQUAL "esp32s31"))
    message(FATAL_ERROR "The rtthread component supports only ESP32-C3, ESP32-C6 and ESP32-S31")
endif()

# Upstream RT-Thread is the only external source tree. Its src/ and include/ are
# used unmodified; everything target specific (rtconfig.h, the CPU port, the
# board init and the stack adapter) lives in this component under port/.
set(root_help "Set RTTHREAD_ROOT to the upstream RT-Thread checkout, using -DRTTHREAD_ROOT=<path> or an environment variable.")
if(DEFINED CACHE{RTTHREAD_ROOT})
    get_property(root_value CACHE RTTHREAD_ROOT PROPERTY VALUE)
else()
    set(root_value "$ENV{RTTHREAD_ROOT}")
endif()
if("${root_value}" STREQUAL "")
    message(FATAL_ERROR "RTTHREAD_ROOT is not set. ${root_help}")
endif()
get_filename_component(root_value "${root_value}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
set(RTTHREAD_ROOT "${root_value}" CACHE PATH "RT-Thread checkout" FORCE)

# RTTHREAD_PORT_ROOT used to point at a forked per-target port checkout that
# duplicated the kernel sources and headers. Those copies are unused: the kernel
# compiles from RTTHREAD_ROOT and the port lives in components/rtthread/port/.
# The variable is still accepted so existing environments and stale CMake cache
# entries keep working; nothing under it is compiled or included any more.
if(DEFINED CACHE{RTTHREAD_PORT_ROOT})
    get_property(port_root_value CACHE RTTHREAD_PORT_ROOT PROPERTY VALUE)
else()
    set(port_root_value "$ENV{RTTHREAD_PORT_ROOT}")
endif()
if(NOT "${port_root_value}" STREQUAL "")
    message(STATUS "rtthread: ignoring deprecated RTTHREAD_PORT_ROOT (${port_root_value}); "
                   "port sources and rtconfig.h now live in components/rtthread/port/.")
endif()

# Kernel and klibc sources compile straight from the upstream checkout; there
# are no local copies. src/ipc.c is needed on both targets: with C3's
# configuration only for the suspend-list helpers, with C6's also for the
# semaphore and mutex objects.
set(kernel_srcs
    "${RTTHREAD_ROOT}/src/components.c"
    "${RTTHREAD_ROOT}/src/clock.c"
    "${RTTHREAD_ROOT}/src/cpu_up.c"
    "${RTTHREAD_ROOT}/src/defunct.c"
    "${RTTHREAD_ROOT}/src/idle.c"
    "${RTTHREAD_ROOT}/src/ipc.c"
    "${RTTHREAD_ROOT}/src/irq.c"
    "${RTTHREAD_ROOT}/src/kservice.c"
    "${RTTHREAD_ROOT}/src/mem.c"
    "${RTTHREAD_ROOT}/src/object.c"
    "${RTTHREAD_ROOT}/src/scheduler_comm.c"
    "${RTTHREAD_ROOT}/src/scheduler_up.c"
    "${RTTHREAD_ROOT}/src/thread.c"
    "${RTTHREAD_ROOT}/src/timer.c"
    "${RTTHREAD_ROOT}/src/klibc/kerrno.c"
    "${RTTHREAD_ROOT}/src/klibc/kstdio.c"
    "${RTTHREAD_ROOT}/src/klibc/kstring.c"
    "${RTTHREAD_ROOT}/src/klibc/rt_vsnprintf_tiny.c"
    "${RTTHREAD_ROOT}/src/klibc/rt_vsscanf.c")

# RT-Thread master split the RISC-V CPU port into libcpu/risc-v/common (the new,
# FDT/DM aware port) and libcpu/risc-v/legacy_common (the flat layout this
# component was written against). The port here needs cpuport.h and
# rt_hw_stack_frame.h at the top of the directory, which is the legacy_common
# layout, so resolve the directory once and use it for both the existence check
# and the include path.
set(rtt_riscv_port_dir "${RTTHREAD_ROOT}/libcpu/risc-v/legacy_common")
if(NOT EXISTS "${rtt_riscv_port_dir}/cpuport.h")
    set(rtt_riscv_port_dir "${RTTHREAD_ROOT}/libcpu/risc-v/common")
endif()

# Check every compiled kernel source and the public/private header entry points.
# context_gcc.S is retained as the reference for the adapted assembly in port/.
foreach(required_file IN ITEMS ${kernel_srcs}
        "${RTTHREAD_ROOT}/include/rtthread.h"
        "${RTTHREAD_ROOT}/include/rthw.h"
        "${rtt_riscv_port_dir}/context_gcc.S"
        "${rtt_riscv_port_dir}/cpuport.h"
        "${rtt_riscv_port_dir}/rt_hw_stack_frame.h")
    if(NOT EXISTS "${required_file}")
        message(FATAL_ERROR "RT-Thread source file is missing: ${required_file}. ${root_help}")
    endif()
endforeach()

# Sources shared by both targets.
set(common_srcs
    "port/common/context_gcc.S"
    "rtthread_stack_init.c")

if(CONFIG_IDF_TARGET_ESP32C3)
    set(target_srcs
        "port/esp32c3/cpuport.c"
        "port/esp32c3/interrupt.c")
    # rtconfig.h is the only header here; the adjacent port C sources cannot
    # shadow upstream kernel headers.
    set(rtt_config_dir "${CMAKE_CURRENT_LIST_DIR}/port/esp32c3")
elseif(CONFIG_IDF_TARGET_ESP32C6)
    set(target_srcs
        "port/esp32c6/cpuport.c")
    set(rtt_config_dir "${CMAKE_CURRENT_LIST_DIR}/port/esp32c6")
elseif(CONFIG_IDF_TARGET_ESP32S31)
    set(target_srcs
        "port/esp32s31/cpuport.c")
    set(rtt_config_dir "${CMAKE_CURRENT_LIST_DIR}/port/esp32s31")
endif()

# These flags are applied identically on both targets: the kernel, the shared
# stack adapter, the context assembly and the CPU port. port/common/board.c
# keeps the component defaults because it calls into IDF.
set_source_files_properties(${kernel_srcs} ${common_srcs} ${target_srcs} PROPERTIES
    COMPILE_DEFINITIONS "__RTTHREAD__;__RT_KERNEL_SOURCE__"
    COMPILE_OPTIONS "-ffreestanding;-fno-builtin")

set(srcs ${kernel_srcs} ${common_srcs} ${target_srcs} "port/common/board.c")

idf_component_register(SRCS ${srcs}
    # Upstream headers first, then this component's per-target config directory
    # (rtconfig.h) and cpuport.h / rt_hw_stack_frame.h from upstream libcpu.
    INCLUDE_DIRS "${RTTHREAD_ROOT}/include" "${rtt_config_dir}"
    PRIV_INCLUDE_DIRS "${rtt_riscv_port_dir}" "${IDF_PATH}/components/freertos"
    PRIV_REQUIRES freertos riscv
    LDFRAGMENTS "linker.lf")

# IDF applies these properties via generator expressions, including to components
# registered earlier (startup, the FreeRTOS tick/port adapters, and applications).
# rtconfig.h must stay globally visible because those include <rtthread.h>.
idf_build_set_property(INCLUDE_DIRECTORIES
    "${RTTHREAD_ROOT}/include;${rtt_config_dir}" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS RT_USING_USER_MAIN APPEND)
# RT-Thread invokes the application's main from a kernel archive. Keep that
# entry point regardless of the component archive order (as IDF does for app_main).
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u main")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    RT_MAIN_THREAD_STACK_SIZE=CONFIG_ESP_MAIN_TASK_STACK_SIZE)
