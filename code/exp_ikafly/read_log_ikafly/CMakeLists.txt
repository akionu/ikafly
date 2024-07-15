add_executable(read_log_ikafly
        read_log_ikafly.cpp
        )

target_link_libraries(read_log_ikafly
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
)

pico_enable_stdio_usb(read_log_ikafly 1)
pico_enable_stdio_uart(read_log_ikafly 0)

pico_add_extra_outputs(read_log_ikafly)
