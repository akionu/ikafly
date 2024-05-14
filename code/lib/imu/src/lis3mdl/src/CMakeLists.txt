
add_library(lis3mdl INTERFACE)

target_sources(lis3mdl INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/lis3mdl.c
    ${CMAKE_CURRENT_LIST_DIR}/lis3mdl-pid/lis3mdl_reg.c
)

target_link_libraries(lis3mdl
    INTERFACE
    pico_stdlib
    hardware_i2c
)
