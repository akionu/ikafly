add_executable(motor_ex motor_ex.c)

target_link_libraries(motor_ex
        pico_stdlib
        hardware_pwm
		motor
)

pico_enable_stdio_usb(motor_ex 1)
pico_enable_stdio_uart(motor_ex 0)

pico_add_extra_outputs(motor_ex)
