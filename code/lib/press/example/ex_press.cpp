#include <stdio.h>
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../src/uprs.h"
#include "pico/time.h"

#define I2C i2c1

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;

Press prs(I2C, 0x77);

int main(void) {
	stdio_usb_init();
	sleep_ms(1000);

	i2c_init(I2C, 400*1000);
	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);

	prs.init();

	float press_hpa, alt_cm, temp_degc;
	printf("Time(ms) Pressure(hPa) Altitude(m) Temperature(degC)\n");
	while (1) {
		press_hpa = prs.getPressHpa();
		alt_cm = prs.getAltM();
		temp_degc = prs.getTempDegC();

		printf("%d %4.2f %4.2f %2.1f\n", time_us_32() / 1000, press_hpa, alt_cm, temp_degc);
		sleep_ms(300);
	}
}
