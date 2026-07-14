cmake_minimum_required(VERSION 3.22)

#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MPS_MotorDriver)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add at32_workbench generated sources
add_subdirectory(cmake/at32_workbench)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_SOURCE_DIR}/application/motor_control/motor_control.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/foc_core.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/current_loop.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/current_reconstruction.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/speed_loop.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/position_loop.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/motor_control_isr.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/fault_manager.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/motor_calibration.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/encoder_service.c
    ${CMAKE_SOURCE_DIR}/application/motor_control/encoder_tracker.c
    ${CMAKE_SOURCE_DIR}/application/motor_app.c
    ${CMAKE_SOURCE_DIR}/application/motor_shell.c
    ${CMAKE_SOURCE_DIR}/middlewares/msp/ma600/ma600a.c
    ${CMAKE_SOURCE_DIR}/middlewares/msp/ma600/ma600a_at32_spi2.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/motor_pwm_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/current_sense_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/motor_encoder_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/encoder_acq_timer_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/motor_protect_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/flash_calibration_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/clock_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/board_init_at32m412.c
    ${CMAKE_SOURCE_DIR}/platform/at32m412/board_usart1_dma.c
    ${CMAKE_SOURCE_DIR}/communication/can_protocol.c
    ${CMAKE_SOURCE_DIR}/communication/can_at32m412.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/application
    ${CMAKE_SOURCE_DIR}/application/motor_control
    ${CMAKE_SOURCE_DIR}/middlewares/msp/ma600
    ${CMAKE_SOURCE_DIR}/platform/at32m412
    ${CMAKE_SOURCE_DIR}/communication
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    at32_workbench

    # Add user defined libraries
)

