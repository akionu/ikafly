/*
 ******************************************************************************
 * @file    read_data_simple.c -> lis3mdl_ex.c
 * @author  Sensors Software Solution Team
 * @author  Yudetamago-AM
 * @brief   This file show the simplest way to get data from sensor.
 * 			Modified to use with RP2040.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../src/lis3mdl.h"

#define I2C i2c1
#define SDA 14
#define SCL 15

int main(void) {
	stdio_init_all();
	sleep_ms(2000);
	printf("This is lis3mdl_ex.\n");

	// I2C init
	i2c_init(I2C, 400 * 1000);
	gpio_set_function(SDA, GPIO_FUNC_I2C);
	gpio_set_function(SCL, GPIO_FUNC_I2C);

	stmdev_ctx_t lis3mdl;
	lis3mdl.write_reg = lis3mdl_i2c_write;
	lis3mdl.read_reg  = lis3mdl_i2c_read;
	lis3mdl.mdelay    = lis3mdl_delay;
	lis3mdl.handle    = I2C;

	uint8_t whoami = 0;
	lis3mdl_device_id_get(&lis3mdl, &whoami);
	if (whoami != LIS3MDL_ID) {
		printf("LIS3MDL not found. whoami: 0x%02x\n", whoami);
		while (1) tight_loop_contents();
	} else {
		printf("LIS3MDL found.");
	}

	uint8_t reset = 0;
	lis3mdl_reset_set(&lis3mdl, PROPERTY_ENABLE);
	do {
		lis3mdl_reset_get(&lis3mdl, &reset);
	} while (reset);

	/* Enable Block Data Update */
	lis3mdl_block_data_update_set(&lis3mdl, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lis3mdl_data_rate_set(&lis3mdl, LIS3MDL_HP_1Hz25);
	/* Set full scale */
	lis3mdl_full_scale_set(&lis3mdl, LIS3MDL_16_GAUSS);
	/* Enable temperature sensor */
	lis3mdl_temperature_meas_set(&lis3mdl, PROPERTY_ENABLE);
	/* Set device in continuous mode */
	lis3mdl_operating_mode_set(&lis3mdl, LIS3MDL_CONTINUOUS_MODE);

	/* Read samples in polling mode (no int) */

	int16_t data_raw_magnetic[3];
	int16_t data_raw_temperature;
	float magnetic_mG[3];
	float temperature_degC;

	while (1) {
		uint8_t reg;
		/* Read output only if new value is available */
		lis3mdl_mag_data_ready_get(&lis3mdl, &reg);

		if (reg) {
			/* Read magnetic field data */
			memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
			lis3mdl_magnetic_raw_get(&lis3mdl, data_raw_magnetic);
			magnetic_mG[0] = 1000 * lis3mdl_from_fs16_to_gauss(
					data_raw_magnetic[0]);
			magnetic_mG[1] = 1000 * lis3mdl_from_fs16_to_gauss(
					data_raw_magnetic[1]);
			magnetic_mG[2] = 1000 * lis3mdl_from_fs16_to_gauss(
					data_raw_magnetic[2]);
			printf("Magnetic field [mG]:%4.2f %4.2f %4.2f\n",
					magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);

			/* Read temperature data */
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
			lis3mdl_temperature_raw_get(&lis3mdl, &data_raw_temperature);
			temperature_degC = lis3mdl_from_lsb_to_celsius(data_raw_temperature);
			printf("Temperature [degC]:%6.2f\n",
					temperature_degC);
		}
		sleep_ms(100);
	}
}
