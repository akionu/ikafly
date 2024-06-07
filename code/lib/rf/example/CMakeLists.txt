add_executable(ex_rf_send ex_rf_send.cpp)

target_link_libraries(ex_rf_send
	pico_stdlib
	rf
)

pico_enable_stdio_usb(ex_rf_send 1)
pico_enable_stdio_uart(ex_rf_send 0)

pico_add_extra_outputs(ex_rf_send)

add_executable(ex_rf_receive ex_rf_receive.cpp)

target_link_libraries(ex_rf_receive
	pico_stdlib
	rf
)

pico_enable_stdio_usb(ex_rf_receive 1)
pico_enable_stdio_uart(ex_rf_receive 0)

pico_add_extra_outputs(ex_rf_receive)
