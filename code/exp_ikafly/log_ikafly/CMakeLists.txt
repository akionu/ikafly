add_executable(log_ikafly
        log_ikafly.cpp
        )

target_link_libraries(log_ikafly
	pico_stdlib
	pico_flash
	hardware_flash
    littlefs
    pico_multicore
    pico_platform
    hardware_uart
    freertos
    hardware_i2c
    hardware_pwm
    gnss
)

pico_enable_stdio_usb(log_ikafly 1)
pico_enable_stdio_uart(log_ikafly 0)

pico_add_extra_outputs(log_ikafly)
