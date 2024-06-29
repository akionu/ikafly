add_executable(ex_log_dirty
        ex_log_dirty.cpp
        )

target_link_libraries(ex_log_dirty
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
    spiffs
)

pico_enable_stdio_usb(ex_log_dirty 1)
pico_enable_stdio_uart(ex_log_dirty 0)

pico_add_extra_outputs(ex_log_dirty)
