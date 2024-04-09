add_executable(wakeup_seq
        wakeup_seq.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(wakeup_seq pico_stdlib hardware_uart)

# enable stdio over usb
pico_enable_stdio_usb(wakeup_seq 1)
pico_enable_stdio_uart(wakeup_seq 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(wakeup_seq)
