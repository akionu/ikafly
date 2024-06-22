cmake_minimum_required(VERSION 3.12)

# set board with 4MB flash
set(PICO_BOARD pimoroni_picolipo_4mb)

# set FreeRTOS Kernel Path
set(FREERTOS_SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/freertos)
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/freertos/FreeRTOS-Kernel)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(ikafly C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# FreeRTOS Settings
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
INTERFACE
    lib/freertos/config
)
target_compile_definitions(freertos_config
  INTERFACE
    projCOVERAGE_TEST=0
	# need for safe flash access
	PICO_FLASH_SAFE_EXECUTE_PICO_SUPPORT_MULTICORE_LOCKOUT=0
	PICO_FLASH_SAFE_EXECUTE_USE_FREERTOS_SMP=1
)

add_subdirectory(exp)
add_subdirectory(lib)
add_subdirectory(freertos_ikafly)
add_subdirectory(exp_ikafly)
add_subdirectory(flight)

