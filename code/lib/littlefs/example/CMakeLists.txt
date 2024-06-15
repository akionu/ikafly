add_executable(littlefs_ex littlefs_ex.cpp)

target_link_libraries(littlefs_ex 
	pico_stdlib
	hardware_i2c
	freertos
	littlefs
)

target_include_directories(littlefs_ex PRIVATE
	../../freertos/*
	../src/*
)
pico_enable_stdio_usb(littlefs_ex 1)
pico_enable_stdio_uart(littlefs_ex 0)

pico_add_extra_outputs(littlefs_ex)
