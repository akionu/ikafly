add_executable(ex1_w_cam
        ex1_w_cam.cpp
        )

target_link_libraries(ex1_w_cam
	pico_stdlib
    tjpg
    hardware_i2c
    cam
)

pico_enable_stdio_usb(ex1_w_cam 1)
pico_enable_stdio_uart(ex1_w_cam 0)

pico_add_extra_outputs(ex1_w_cam)
