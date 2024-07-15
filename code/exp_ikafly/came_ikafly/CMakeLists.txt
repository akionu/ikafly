add_executable(came_ikafli
        came_ikafli.cpp
        )

target_link_libraries(came_ikafli
	pico_stdlib
    tjpg
    hardware_i2c
    cam
)

pico_enable_stdio_usb(came_ikafli 1)
pico_enable_stdio_uart(ex2_came_ikafli 0)

pico_add_extra_outputs(came_ikafli)
