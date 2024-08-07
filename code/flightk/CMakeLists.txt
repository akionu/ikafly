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

add_executable(flight_read
    flightread.cpp
    ulog.cpp
)

target_link_libraries(flight_read
    pico_stdlib
    pico_flash
    hardware_rtc
    littlefs
)

pico_enable_stdio_usb(flight_read 1)
pico_enable_stdio_uart(flight_read 0)
pico_add_extra_outputs(flight_read)
