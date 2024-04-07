add_executable(lis3mdl_ex lis3mdl_ex.c)

target_link_libraries(lis3mdl_ex pico_stdlib hardware_i2c lis3mdl)

pico_enable_stdio_usb(lis3mdl_ex 1)
pico_enable_stdio_uart(lis3mdl_ex 0)

pico_add_extra_outputs(lis3mdl_ex)
