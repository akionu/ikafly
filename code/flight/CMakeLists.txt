add_compile_definitions(
    LFS_THREADSAFE
)

set(TargetName flight2.cpp)

add_executable(${TargetName}
    flight2.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    pico_multicore
    pico_platform
    hardware_uart
    freertos
    hardware_i2c
    uprs
    hardware_pwm
    gnss
    imu
    rf
    umotor
    littlefs
    tjpg
    cam
)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
