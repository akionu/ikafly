set(TargetName test_landing.cpp)

add_executable(${TargetName}
    test_landing.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    pico_multicore
    pico_platform
    hardware_uart
    hardware_i2c
    uprs
    hardware_pwm
    freertos
    pico_flash
	hardware_flash
    littlefs

)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
