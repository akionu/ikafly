add_executable(cam_ex_cpp
        cam_ex.cpp
        )

target_link_libraries(cam_ex_cpp pico_stdlib hardware_i2c cam)

pico_enable_stdio_usb(cam_ex_cpp 1)
pico_enable_stdio_uart(cam_ex_cpp 0)

pico_add_extra_outputs(cam_ex_cpp)

