add_executable(flash_nuke_4MB
        flash_nuke_4MB.cpp
        )

target_link_libraries(flash_nuke_4MB pico_stdlib hardware_flash)

pico_enable_stdio_usb(flash_nuke_4MB 1)
pico_enable_stdio_uart(flash_nuke_4MB 0)

pico_add_extra_outputs(flash_nuke_4MB)
