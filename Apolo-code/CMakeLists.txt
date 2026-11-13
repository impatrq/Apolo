# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated CMake Pico project file, adapted for FreeRTOS.
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)
project(apolo2 C CXX ASM)
pico_sdk_init()

# FREERTOS_KERNEL_PATH debe apuntar al repositorio FreeRTOS-Kernel.
include(FreeRTOS_Kernel_import.cmake)

add_executable(apolo2 apolo2.c)
pico_set_program_name(apolo2 "apolo2")
pico_set_program_version(apolo2 "0.2")

pico_enable_stdio_uart(apolo2 0)
pico_enable_stdio_usb(apolo2 1)

target_compile_definitions(apolo2 PRIVATE configNUMBER_OF_CORES=1)
target_include_directories(apolo2 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(apolo2 PRIVATE
    pico_stdlib
    hardware_clocks
    hardware_pwm
    hardware_adc
    hardware_gpio
    FreeRTOS-Kernel-Heap4
)

pico_add_extra_outputs(apolo2)
