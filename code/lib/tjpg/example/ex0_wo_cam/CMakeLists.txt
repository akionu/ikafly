add_executable(ex0_wo_cam
        ex0_wo_cam.cpp
        )

target_link_libraries(ex0_wo_cam
	pico_stdlib
    tjpg
)

pico_enable_stdio_usb(ex0_wo_cam 1)
pico_enable_stdio_uart(ex0_wo_cam 0)

pico_add_extra_outputs(ex0_wo_cam)
