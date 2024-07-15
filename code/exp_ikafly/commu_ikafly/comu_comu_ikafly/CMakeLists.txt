set(TargetName comu_comu.cpp)


add_executable(${TargetName}
    comu_comu.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    pico_multicore
    pico_platform
    hardware_uart
    hardware_i2c
    hardware_pwm
    freertos
    rf


)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
