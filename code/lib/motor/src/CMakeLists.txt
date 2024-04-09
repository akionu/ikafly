add_library(motor INTERFACE)


target_sources(motor INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/motor.c
)

target_link_libraries(motor
        INTERFACE
        pico_stdlib
        hardware_pwm
)
