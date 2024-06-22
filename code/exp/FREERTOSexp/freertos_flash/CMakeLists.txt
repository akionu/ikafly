add_executable(flash_rtos
        flash_rtos.cpp
        )

target_link_libraries(flash_rtos
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
)

pico_enable_stdio_usb(flash_rtos 1)
pico_enable_stdio_uart(flash_rtos 0)

pico_add_extra_outputs(flash_rtos)
