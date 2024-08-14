#add_compile_definitions(
#    LFS_THREADSAFE
#)

add_library(common_lib STATIC)
target_link_libraries(common_lib
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

file(GLOB srcs
    ugps.cpp
    ulog.cpp
    angle.cpp
    flightk.cpp
)


# no.1
add_executable(kflight_1 ${srcs})
target_link_libraries(kflight_1 common_lib)
pico_enable_stdio_usb(kflight_1 1)
pico_enable_stdio_uart(kflight_1 0)
pico_add_extra_outputs(kflight_1)

# no.2
add_executable(kflight_2 ${srcs})
target_link_libraries(kflight_2 common_lib)
pico_enable_stdio_usb(kflight_2 1)
pico_enable_stdio_uart(kflight_2 0)
pico_add_extra_outputs(kflight_2)

# no.3
add_executable(kflight_3 ${srcs})
target_link_libraries(kflight_3 common_lib)
pico_enable_stdio_usb(kflight_3 1)
pico_enable_stdio_uart(kflight_3 0)
pico_add_extra_outputs(kflight_3)

add_executable(kflight_4 ${srcs})
target_link_libraries(kflight_4 common_lib)
pico_enable_stdio_usb(kflight_4 1)
pico_enable_stdio_uart(kflight_4 0)
pico_add_extra_outputs(kflight_4)

add_executable(kflight_5 ${srcs})
target_link_libraries(kflight_5 common_lib)
pico_enable_stdio_usb(kflight_5 1)
pico_enable_stdio_uart(kflight_5 0)
pico_add_extra_outputs(kflight_5)

add_executable(kflight_6 ${srcs})
target_link_libraries(kflight_6 common_lib)
pico_enable_stdio_usb(kflight_6 1)
pico_enable_stdio_uart(kflight_6 0)
pico_add_extra_outputs(kflight_6)

add_executable(kflight_7 ${srcs})
target_link_libraries(kflight_7 common_lib)
pico_enable_stdio_usb(kflight_7 1)
pico_enable_stdio_uart(kflight_7 0)
pico_add_extra_outputs(kflight_7)

add_executable(kflight_8 ${srcs})
target_link_libraries(kflight_8 common_lib)
pico_enable_stdio_usb(kflight_8 1)
pico_enable_stdio_uart(kflight_8 0)
pico_add_extra_outputs(kflight_8)

add_executable(kflight_9 ${srcs})
target_link_libraries(kflight_9 common_lib)
pico_enable_stdio_usb(kflight_9 1)
pico_enable_stdio_uart(kflight_9 0)
pico_add_extra_outputs(kflight_9)

add_executable(kflight_10 ${srcs})
target_link_libraries(kflight_10 common_lib)
pico_enable_stdio_usb(kflight_10 1)
pico_enable_stdio_uart(kflight_10 0)
pico_add_extra_outputs(kflight_10)

# SETTINGS
target_compile_definitions(kflight_1 PUBLIC CODE=1 MLEFT=-1 MRIGHT=1)
target_compile_definitions(kflight_2 PUBLIC CODE=2 MLEFT=-1 MRIGHT=1)
target_compile_definitions(kflight_3 PUBLIC CODE=3 MLEFT=-1 MRIGHT=1)
target_compile_definitions(kflight_4 PUBLIC CODE=4 MLEFT=-1 MRIGHT=1)
target_compile_definitions(kflight_5 PUBLIC CODE=5 MLEFT=-1 MRIGHT=1)
target_compile_definitions(kflight_6 PUBLIC CODE=6 MLEFT=-1 MRIGHT=1)
target_compile_definitions(kflight_7 PUBLIC CODE=7 MLEFT=-1 MRIGHT=1)
target_compile_definitions(kflight_8 PUBLIC CODE=8 MLEFT=1 MRIGHT=1)
target_compile_definitions(kflight_9 PUBLIC CODE=9 MLEFT=-1 MRIGHT=1)
target_compile_definitions(kflight_10 PUBLIC CODE=10 MLEFT=-1 MRIGHT=1)

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

add_executable(calibration
    calibration.cpp
    ugps.cpp
    angle.cpp
    ulog.cpp
)

target_link_libraries(calibration common_lib)
pico_enable_stdio_usb(calibration 1)
pico_enable_stdio_uart(calibration 0)
pico_add_extra_outputs(calibration)
