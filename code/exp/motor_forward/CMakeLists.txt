add_executable(motor_forward_cpp motor_forward.cpp)

target_link_libraries(motor_forward_cpp
	pico_stdlib
	umotor
)

pico_enable_stdio_usb(motor_forward_cpp 1)
pico_enable_stdio_uart(motor_forward_cpp 0)

pico_add_extra_outputs(motor_forward_cpp)
