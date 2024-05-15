# add_subdirectory(FreeRTOS-Kernel)

add_library(freertos)
target_include_directories(freertos PUBLIC
    ${FREERTOS_KERNEL_PATH}
)
target_link_libraries(freertos
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    pico_stdlib
)