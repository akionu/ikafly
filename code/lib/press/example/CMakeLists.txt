add_executable(ex_press ex_press.cpp)

target_link_libraries(ex_press
	pico_stdlib
	hardware_i2c
	uprs
)

pico_enable_stdio_usb(ex_press 1)
pico_enable_stdio_uart(ex_press 0)

pico_add_extra_outputs(ex_press)
