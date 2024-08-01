add_executable(uart_read
        uart_read.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(uart_read pico_stdlib hardware_uart)

# enable stdio over usb
pico_enable_stdio_usb(uart_read 1)
pico_enable_stdio_uart(uart_read 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(uart_read)

add_executable(uart_read_gps
        uart_read_gps.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(uart_read_gps pico_stdlib hardware_uart)

# enable stdio over usb
pico_enable_stdio_usb(uart_read_gps 1)
pico_enable_stdio_uart(uart_read_gps 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(uart_read_gps)
