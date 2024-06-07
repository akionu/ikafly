
set(TargetName freertos_gnss)

add_executable(${TargetName}
    freertos_gnss.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    pico_multicore
    freertos
    hardware_uart
    gnss
)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
