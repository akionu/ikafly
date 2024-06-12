set(TargetName radio)

add_executable(${TargetName}
    radio.cpp
)

target_link_libraries(${TargetName} 
    pico_stdlib 
    hardware_uart
    gnss
    rf
)

pico_enable_stdio_usb(${TargetName} 1)
pico_enable_stdio_uart(${TargetName} 0)

pico_add_extra_outputs(${TargetName})
