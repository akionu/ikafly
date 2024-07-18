set(TargetName exp_landing)

add_compile_definitions(LFS_THREADSAFE)

add_executable(${TargetName}
    exp_landing.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    hardware_uart
    freertos
    littlefs
    hardware_i2c
    uprs
    imu
)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
