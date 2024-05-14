/*
 ******************************************************************************
 * @file    _offset.c -> lsm6dso_offset.c
 * @author  Sensors Software Solution Team, akionu(modified for RP2040)
 * @brief   This file show the simplest way to get data from sensor.
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

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "../src/lsm6dso.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C i2c1
#define SDA 14
#define SCL 15

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;

uint8_t twos_compl_neg(uint8_t n) {
	n ^= 0xff;
	n += 1;
	return ((uint8_t)n);
}

/* Main Example --------------------------------------------------------------*/
int main(void)
{
	stdio_init_all();
	sleep_ms(2000);
	printf("This is lis3mdl_ex.\n");

	// I2C init
	i2c_init(I2C, 400 * 1000);
	gpio_set_function(SDA, GPIO_FUNC_I2C);
	gpio_set_function(SCL, GPIO_FUNC_I2C);
	stmdev_ctx_t dev_ctx;

	/* Example of XL offset to apply to acc. output */
	uint8_t offset[3] = { 48, 64, twos_compl_neg(127)};
	printf("offset: %d %d %d\n", offset[0], offset[1], offset[2]);
	/* Initialize mems driver interface */
	dev_ctx.write_reg = lsm6dso_i2c_write;
	dev_ctx.read_reg = lsm6dso_i2c_read;
	dev_ctx.mdelay = lsm6dso_delay;
	dev_ctx.handle = I2C;
	/* Check device ID */
	lsm6dso_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != LSM6DSO_ID)
		while (1);

	/* Restore default configuration */
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lsm6dso_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Disable I3C interface */
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
	/* Enable Block Data Update */
	lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Weight of XL user offset to 2^(-10) g/LSB */
	lsm6dso_xl_offset_weight_set(&dev_ctx, LSM6DSO_LSb_1mg);
	/* Accelerometer X,Y,Z axis user offset correction expressed
	 * in two’s complement. Set X to 48mg, Y tp 64 mg, Z to -127 mg
	 */
	lsm6dso_xl_usr_offset_x_set(&dev_ctx, &offset[0]);
	lsm6dso_xl_usr_offset_y_set(&dev_ctx, &offset[1]);
	lsm6dso_xl_usr_offset_z_set(&dev_ctx, &offset[2]);
	lsm6dso_xl_usr_offset_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);
	/* Set full scale */
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);
	/* Configure filtering chain(No aux interface). */
	/* Accelerometer - LPF1 + LPF2 path */
	lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

	/* Read samples in polling mode (no int). */
	while (1) {
		uint8_t reg;
		/* Read output only if new xl value is available */
		lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);

		if (reg) {
			/* Read acceleration field data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
			acceleration_mg[0] =
				lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
			acceleration_mg[1] =
				lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
			acceleration_mg[2] =
				lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);
			printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
					acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		}

		lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);

		if (reg) {
			/* Read angular rate field data */
			memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
			lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
			angular_rate_mdps[0] =
				lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[0]);
			angular_rate_mdps[1] =
				lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[1]);
			angular_rate_mdps[2] =
				lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[2]);
			printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
					angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
		}

		lsm6dso_temp_flag_data_ready_get(&dev_ctx, &reg);

		if (reg) {
			/* Read temperature data. */
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
			lsm6dso_temperature_raw_get(&dev_ctx, &data_raw_temperature);
			temperature_degC = lsm6dso_from_lsb_to_celsius(
					data_raw_temperature);
			printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
		}
	}
}

