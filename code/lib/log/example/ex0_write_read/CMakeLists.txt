add_executable(ex0_write_read
        ex0_write_read.cpp
        )

target_link_libraries(ex0_write_read
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
    log
)

target_include_directories(ex0_write_read PUBLIC
    ../src/*
    {$FREERTOS_KERNE_PATH}/
)

pico_enable_stdio_usb(ex0_write_read 1)
pico_enable_stdio_uart(ex0_write_read 0)

pico_add_extra_outputs(ex0_write_read)
