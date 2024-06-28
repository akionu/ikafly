add_executable(ex1_boot_count
        ex1_boot_count.cpp
        )

target_link_libraries(ex1_boot_count
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
    log
)

target_include_directories(ex1_boot_count PUBLIC
    ../src/*
    {$FREERTOS_KERNE_PATH}/
)

pico_enable_stdio_usb(ex1_boot_count 1)
pico_enable_stdio_uart(ex1_boot_count 0)

pico_add_extra_outputs(ex1_boot_count)
