add_executable(selftest
        selftest.cpp
        )

target_link_libraries(selftest 
		pico_stdlib
		hardware_i2c
		hardware_uart
		hardware_pwm
		hardware_pio
		uprs
		umotor
		imu
		cam
	)

pico_enable_stdio_usb(selftest 1)
pico_enable_stdio_uart(selftest 0)

pico_add_extra_outputs(selftest)
