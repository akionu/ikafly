add_executable(ex_flash
        ex_flash.cpp
        )

target_link_libraries(ex_flash pico_stdlib hardware_flash)

pico_enable_stdio_usb(ex_flash 1)
pico_enable_stdio_uart(ex_flash 0)

pico_add_extra_outputs(ex_flash)
