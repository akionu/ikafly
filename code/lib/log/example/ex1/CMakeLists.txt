add_executable(ex1_log
        ex1_log.cpp
        )

target_link_libraries(ex1_log
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
    log
)

target_include_directories(ex1_log PUBLIC
    ../src/*
    {$FREERTOS_KERNE_PATH}/
)

pico_enable_stdio_usb(ex1_log 1)
pico_enable_stdio_uart(ex1_log 0)

pico_add_extra_outputs(ex1_log)
