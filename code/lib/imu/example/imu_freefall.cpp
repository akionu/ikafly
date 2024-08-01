

#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../src/imu.h"

#define I2C i2c1
#define SDA 14
#define SCL 15


IMU imu(I2C);

int main(void) {
	stdio_init_all();
	sleep_ms(2000);
	printf("This is %s\n", __FILE_NAME__);

	// I2C init
	i2c_init(I2C, 400 * 1000);
	gpio_set_function(SDA, GPIO_FUNC_I2C);
	gpio_set_function(SCL, GPIO_FUNC_I2C);

	imu.setDebugPrint(false);

	bool st = imu.init();
	if (st) printf("IMU init success");
	else printf("IMU init fails");

	while (1) {
        if (imu.isFreeFallNow()) printf("%d: FF!\n", time_us_32()/1000);
	}

}
