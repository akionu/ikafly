add_executable(ex2_cone
        ex2_cone.cpp
        )

target_link_libraries(ex2_cone
	pico_stdlib
    tjpg
    hardware_i2c
    cam
)

pico_enable_stdio_usb(ex2_cone 1)
pico_enable_stdio_uart(ex2_cone 0)

pico_add_extra_outputs(ex2_cone)
