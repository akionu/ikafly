add_library(pico-spl06-001 INTERFACE)

target_sources(pico-spl06-001 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico-spl06-001.c
)

target_link_libraries(pico-spl06-001
    INTERFACE
    pico_stdlib
    hardware_i2c
)