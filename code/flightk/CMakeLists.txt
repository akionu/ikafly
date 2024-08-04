#add_compile_definitions(
#    LFS_THREADSAFE
#)

set(TargetName flightk.cpp)

add_executable(${TargetName}
    ugps.cpp
    ulog.cpp
    flightk.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    pico_flash
    hardware_uart
    hardware_pio
    hardware_dma
    hardware_rtc
    freertos
    hardware_i2c
    uprs
    hardware_pwm
    gnss
    imu
    littlefs
    rf
    cam
    tjpg
    umotor
)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
