add_compile_definitions(LFS_THREADSAFE)

add_executable(exp_landing
    exp_landing.cpp
)


target_link_libraries(exp_landing
    pico_stdlib 
    hardware_uart
    freertos
    littlefs
    hardware_i2c
    uprs
    imu
)

pico_enable_stdio_usb(exp_landing 1)
pico_enable_stdio_uart(exp_landing 0)

pico_add_extra_outputs(exp_landing)


add_executable(exp_landing_read
    exp_landing_read.cpp
)
target_link_libraries(exp_landing_read
    pico_stdlib 
    hardware_uart
    freertos
    littlefs
)

pico_enable_stdio_usb(exp_landing_read 1)
pico_enable_stdio_uart(exp_landing_read 0)

pico_add_extra_outputs(exp_landing_read)
