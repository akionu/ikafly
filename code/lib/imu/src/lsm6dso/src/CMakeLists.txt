
add_library(lsm6dso INTERFACE)

target_sources(lsm6dso INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/lsm6dso.c
    ${CMAKE_CURRENT_LIST_DIR}/lsm6dso-pid/lsm6dso_reg.c
)

target_link_libraries(lsm6dso
    INTERFACE
    pico_stdlib
    hardware_i2c
)
