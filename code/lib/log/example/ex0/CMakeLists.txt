add_executable(ex0_log
        ex0_log.cpp
        )

target_link_libraries(ex0_log
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
    log
)

target_include_directories(ex0_log PUBLIC
    ../src/*
    {$FREERTOS_KERNE_PATH}/
)

pico_enable_stdio_usb(ex0_log 1)
pico_enable_stdio_uart(ex0_log 0)

pico_add_extra_outputs(ex0_log)
