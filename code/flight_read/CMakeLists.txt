add_compile_definitions(
    LFS_THREADSAFE
)

add_executable(flight_read
flight_read.cpp
        )

target_link_libraries(flight_read
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

pico_enable_stdio_usb(flight_read 1)
pico_enable_stdio_uart(flight_read 0)

pico_add_extra_outputs(flight_read)
