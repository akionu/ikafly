
add_compile_definitions(LFS_THREADSAFE)

add_executable(accel_log
    accel_log.cpp
)


target_link_libraries(accel_log
    pico_stdlib 
    hardware_uart
    freertos
    littlefs
    hardware_i2c
    uprs
    imu
)

pico_enable_stdio_usb(accel_log 1)
pico_enable_stdio_uart(accel_log 0)

pico_add_extra_outputs(accel_log)

add_executable(accel_log_read
    accel_log_read.cpp
)


target_link_libraries(accel_log_read
    pico_stdlib 
    hardware_uart
    freertos
    littlefs
    hardware_i2c
    uprs
    imu
)

pico_enable_stdio_usb(accel_log_read 1)
pico_enable_stdio_uart(accel_log_read 0)

pico_add_extra_outputs(accel_log_read)
