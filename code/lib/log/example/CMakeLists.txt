add_executable(ex_log
        ex_log.cpp
        )

target_link_libraries(ex_log
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
    log
)

target_include_directories(ex_log PUBLIC
    ../src/*
    {$FREERTOS_KERNE_PATH}/
)

pico_enable_stdio_usb(ex_log 1)
pico_enable_stdio_uart(ex_log 0)

pico_add_extra_outputs(ex_log)
