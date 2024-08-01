add_executable(selftest2
        selftest.cpp
        )

target_link_libraries(selftest2
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

pico_enable_stdio_usb(selftest2 1)
pico_enable_stdio_uart(selftest2 0)

pico_add_extra_outputs(selftest2)
