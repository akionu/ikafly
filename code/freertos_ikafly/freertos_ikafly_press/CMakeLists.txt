set(TargetName freertos_ikafly_press)

add_executable(${TargetName}
    freertos_ikafly_press.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    pico_multicore
    freertos
    hardware_i2c
    uprs
    hardware_pwm
)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
