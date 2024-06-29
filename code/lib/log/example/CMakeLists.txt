add_executable(ex_lfs
        ex_lfs.cpp
        )

target_link_libraries(ex_lfs
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
    littlefs
)

pico_enable_stdio_usb(ex_lfs 1)
pico_enable_stdio_uart(ex_lfs 0)

pico_add_extra_outputs(ex_lfs)
