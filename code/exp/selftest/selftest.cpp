#include <cstdint>
#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "../../lib/imu/src/imu.h"
#include "../../lib/motor/src/umotor.h"
#include "../../lib/press/src/uprs.h"
#include "../../lib/cam/src/cam.h"
#include "../../lib/ikafly_pin.h"

// i2c sensors
Press prs(bus_i2c, 0x77);
IMU imu(bus_i2c);
Motor motor;
Cam cam(bus_i2c, pin_dvp_exclk, pin_dvp_y2, pio0, 0, 0);

int main(void) {
	stdio_init_all();
	sleep_ms(1000);
	printf("Selftest Ikafly:\n");

	i2c_init(bus_i2c, 400*1000);
	gpio_set_function(pin_i2c1_scl, GPIO_FUNC_I2C);
	gpio_set_function(pin_i2c1_sda, GPIO_FUNC_I2C);

	// pressure
	bool prs_st = prs.init();
	if (!prs_st) printf("press: NG\n");
	else {

		float prs_hpa[5] = {0};
		prs.getPressHpa(); // 最初は捨てる
		for (int8_t i = 0; i < 5; i++) {
			prs_hpa[i] = prs.getPressHpa();
			sleep_ms(300);
		}

		int8_t prs_n = 0;
		for (int8_t i = 1; i < 5; i++) {
			if (prs_hpa[i] != 0 
					&& 800 <= prs_hpa[i] 
					&& prs_hpa[i] <= 1200
					&& prs_hpa[i-1] != prs_hpa[i])
				prs_n++;
		}
		if (prs_n >= 3) printf("press: OK\n");
		else {
			printf("press: NG\n");
			printf("press(hPa): ");
			for (int8_t i = 0; i < 5; i++) {
				printf("%4.2f", prs_hpa[i]);
			}
			printf("\n");
		}
	}

	// imu
	float accel[10][3] = {{0}};
	float gyro[10][3] = {{0}};
	float mag[10][3] = {{0}};
	bool imu_st = imu.init();
	if (!imu_st) printf("IMU: NG\n");
	else {
		for (int8_t i = 0; i < 10; i++) {
			imu.update();
			imu.getAccel_g(accel[i]);
			imu.getGyro_dps(gyro[i]);
			imu.getMag_mG(mag[i]);
			sleep_ms(50);
		}
		int8_t imu_n = 0;
		for (int8_t i = 0; i < 10; i++) {
			if ((accel[i][0] != 0 || accel[i][1] != 0 || accel[i][2] != 0)
					&& (gyro[i][0] != 0 && gyro[i][1] != 0 && gyro[i][2] != 0)
					&& mag[i][0] != 0 && mag[i][1] != 0 && mag[i][2] != 0)
				imu_n++;
		}

		if (imu_n >= 8)	printf("IMU: OK\n");
		else {
			printf("IMU: NG\n");
			for (int8_t i = 0; i < 10; i++) {
				printf("i: %d, "
						"accel: %3.2f %3.2f %3.2f, "
						"gyro: %3.2f %3.2f %3.2f, "
						"mag: %3.2f %3.2f %3.2f\n", 
						i,
						accel[i][0], accel[i][1], accel[i][2],
						gyro[i][0], gyro[i][1], gyro[i][2],
						mag[i][0], mag[i][1], mag[i][2]);
			}
		}
	}

	// camera
	cam.init();
	cam.enableJpeg();
	for (uint16_t i = 0; i < 50; i++) cam.image_buf[i] = 0x00;
	cam.capture();
	while (!cam.isCaptureFinished()) tight_loop_contents();

	int8_t cam_n = 0;
	for (int8_t i = 0; i < 50; i++) {
		if (cam.image_buf[i] != 0x00) cam_n++;
	}
	if (cam_n >= 30) printf("Cam: OK\n");
	else {
		printf("cam: NG\n");
		printf("cam: first 20 bytes: ");
		for (int8_t i = 0; i < 20; i++)
			printf("%02x", cam.image_buf[i]);
		printf("\n");
	}

	// gnss
	gpio_init(pin_gnss_vcc);
	gpio_set_dir(pin_gnss_vcc, GPIO_OUT);
	gpio_put(pin_gnss_vcc, 0);

	uart_init(uart0, 9600);
	gpio_set_function(pin_uart0_miso, GPIO_FUNC_UART);
	gpio_set_function(pin_uart0_mosi, GPIO_FUNC_UART);
	uart_set_baudrate(uart0, 9600);
	uart_set_hw_flow(uart0, false, false);
	uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
	uart_set_fifo_enabled(uart0, true);

	sleep_ms(1000);

	uint8_t buf[100] = {'\0'};
	int16_t n = uart_is_readable(uart0);
	if (n > 0) {
		uart_read_blocking(uart0, buf, 50);
		printf("gnss: OK, n: %d\n", n);
	}
	else printf("gnss: NG, n: %d\n", n);

	// motor
	motor.init(pin_motor1_a, pin_motor2_a);
	motor.setDirForward(1, 1);
	motor.forward(1023);
	sleep_ms(3000);
	motor.stop();
	printf("motor: OK?\n");

	// nichrome
	gpio_init(pin_nichrome_left);
	gpio_set_dir(pin_nichrome_left, GPIO_OUT);
	gpio_put(pin_nichrome_left, 1); // HEAT
	sleep_ms(3000);
	gpio_put(pin_nichrome_left, 0);
	printf("nichrome: OK?\n");
}
