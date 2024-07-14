add_library(cam INTERFACE)

pico_generate_pio_header(cam ${CMAKE_CURRENT_LIST_DIR}/image.pio)

target_sources(cam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/cam.cpp
)

target_link_libraries(cam
    INTERFACE
    pico_stdlib
    hardware_i2c
	hardware_pwm
	hardware_pio
	hardware_dma
)

target_include_directories(cam
    INTERFACE
    .
)
