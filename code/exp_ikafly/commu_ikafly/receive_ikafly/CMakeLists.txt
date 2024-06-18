set(TargetName receive_ikfly.cpp)


add_executable(${TargetName}
    receive_ikafly.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    pico_multicore
    pico_platform
    hardware_uart
    hardware_i2c
    hardware_pwm
    imu
    rf


)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
