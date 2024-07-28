set(TargetName freertos_ikafly_gnss)

add_executable(${TargetName}
    freertos_ikafly_gnss.cpp
)

target_link_libraries(${TargetName} 
    freertos
    pico_multicore
    hardware_pwm
    pico_stdlib
    hardware_uart
    gnss
    
)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
