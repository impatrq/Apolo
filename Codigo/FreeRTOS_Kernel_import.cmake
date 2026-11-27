# The RP2040 FreeRTOS port import helper is included with Pico SDK 2.3.1.
if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH must be set before importing FreeRTOS")
endif()
include("${PICO_SDK_PATH}/test/sync_interop_test/FreeRTOS_Kernel_import.cmake")
