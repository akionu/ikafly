add_executable(lsm6dso_offset lsm6dso_offset.c)

target_link_libraries(lsm6dso_offset pico_stdlib hardware_i2c lsm6dso)

pico_enable_stdio_usb(lsm6dso_offset 1)
pico_enable_stdio_uart(lsm6dso_offset 0)

pico_add_extra_outputs(lsm6dso_offset)
