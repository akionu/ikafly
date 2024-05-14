add_executable(imu_euler imu_euler.cpp)

target_link_libraries(imu_euler 
	pico_stdlib
	hardware_i2c
	imu)

pico_enable_stdio_usb(imu_euler 1)
pico_enable_stdio_uart(imu_euler 0)

pico_add_extra_outputs(imu_euler)
