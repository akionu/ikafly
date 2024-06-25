add_executable(camera_lead_cpp
    camera_lead.cpp)

target_link_libraries(camera_lead_cpp
	pico_stdlib
    hardware_i2c
	cam
    jpeg-9f
    libbmp-master
)

pico_enable_stdio_usb(camera_lead_cpp 1)
pico_enable_stdio_uart(camera_lead_cpp 0)

pico_add_extra_outputs(camera_lead_cpp)
