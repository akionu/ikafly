add_executable(ex_spl06 ex_spl06.c)

target_link_libraries(ex_spl06 pico_stdlib hardware_i2c pico-spl06-001)

pico_enable_stdio_usb(ex_spl06 1)
pico_enable_stdio_uart(ex_spl06 0)

pico_add_extra_outputs(ex_spl06)