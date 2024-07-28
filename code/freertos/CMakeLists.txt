#add_executable(event event.cpp)
add_executable(queue queue.cpp)

#target_link_libraries(event
  #  pico_stdlib
  # pico_flash
  #  hardware_flash
  #  freertos
  #)

#pico_enable_stdio_usb(event 1)
#pico_enable_stdio_uart(event 0)

#pico_add_extra_outputs(event)

target_link_libraries(queue
    pico_stdlib
    freertos
)

pico_enable_stdio_usb(queue 1)
pico_enable_stdio_uart(queue 0)

pico_add_extra_outputs(queue)
