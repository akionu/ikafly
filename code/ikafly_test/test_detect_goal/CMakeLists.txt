add_executable(test_detect_goal
        test_detect_goal.cpp
        )

target_link_libraries(test_detect_goal
	pico_stdlib
    tjpg
    hardware_i2c
    cam
    freertos
    pico_multicore
    pico_platform
    hardware_uart
    freertos
    hardware_pwm
    umotor
    
)

pico_enable_stdio_usb(test_detect_goal 1)
pico_enable_stdio_uart(test_detect_goal 0)

pico_add_extra_outputs(test_detect_goal)
