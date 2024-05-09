add_executable(ex_motor_cpp ex_motor.cpp)

target_link_libraries(ex_motor_cpp
	pico_stdlib
	umotor
)

pico_enable_stdio_usb(ex_motor_cpp 1)
pico_enable_stdio_uart(ex_motor_cpp 0)

pico_add_extra_outputs(ex_motor_cpp)
