cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# AT32 WorkBench generated symbols (macros)
set(WK_Defines_Syms
    USE_STDPERIPH_DRIVER
    AT32M412KBU7_4
    $<$<CONFIG:Debug>:DEBUG>
)

# AT32 WorkBench generated include paths
set(WK_Include_Dirs
    ${CMAKE_SOURCE_DIR}/project/inc
    ${CMAKE_SOURCE_DIR}/libraries/drivers/inc
    ${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/core_support
    ${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/device_support
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/include
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/components/finsh
)

# AT32 WorkBench generated application sources
set(WK_Application_Src
    ${CMAKE_SOURCE_DIR}/project/src/main.c
    ${CMAKE_SOURCE_DIR}/project/src/syscalls.c
    ${CMAKE_SOURCE_DIR}/project/src/sysmem.c
    ${CMAKE_SOURCE_DIR}/project/src/at32m412_416_wk_config.c
    ${CMAKE_SOURCE_DIR}/project/src/at32m412_416_int.c
    ${CMAKE_SOURCE_DIR}/startup_at32m412_416.s
    ${CMAKE_SOURCE_DIR}/project/src/rtthread_app.c
    ${CMAKE_SOURCE_DIR}/project/src/board.c
)

# AT32 Drivers
set(AT32_Drivers_Src
    ${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/device_support/system_at32m412_416.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_crm.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_tmr.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_ertc.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_gpio.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_i2c.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_usart.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_pwc.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_adc.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_spi.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_dma.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_debug.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_flash.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_crc.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_wwdt.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_wdt.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_exint.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_misc.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_scfg.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_can.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_dac.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_cmp.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32m412_416_op.c
)

# Drivers Middlewares
set(RT-Thread_Nano_Src
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/clock.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/components.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/cpu.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/idle.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/ipc.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/irq.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/kservice.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/mem.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/memheap.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/mempool.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/object.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/scheduler.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/signal.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/slab.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/thread.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/src/timer.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/libcpu/arm/cortex-m4/cpuport.c
    ${CMAKE_SOURCE_DIR}/middlewares/rt-thread/libcpu/arm/cortex-m4/context_gcc.s
)

# Link directories setup
set(WK_LINK_DIRS

)

# Project static libraries
set(WK_LINK_LIBS
    AT32_Drivers
    RT-Thread_Nano
)

# Interface library for includes and symbols
add_library(at32_workbench INTERFACE)
target_include_directories(at32_workbench INTERFACE ${WK_Include_Dirs})
target_compile_definitions(at32_workbench INTERFACE ${WK_Defines_Syms})

# Create AT32_Drivers static library
add_library(AT32_Drivers OBJECT)
target_sources(AT32_Drivers PRIVATE ${AT32_Drivers_Src})
target_link_libraries(AT32_Drivers PUBLIC at32_workbench)

# Create RT-Thread_Nano static library
add_library(RT-Thread_Nano OBJECT)
target_sources(RT-Thread_Nano PRIVATE ${RT-Thread_Nano_Src})
target_link_libraries(RT-Thread_Nano PUBLIC at32_workbench)

# Add AT32 WorkBench generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${WK_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${WK_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${WK_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that AT32 WorkBench code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
