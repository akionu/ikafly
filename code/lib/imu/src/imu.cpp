#include "imu.h"
#include "MadgwickAHRS/MadgwickAHRS.h"
#include "FreeRTOSConfig.h"
#include "../../../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../../../lib/freertos/FreeRTOS-Kernel/include/task.h"

IMU::IMU(i2c_inst_t* i2c) {
	madgwick_data.beta = 1.5f;
	madgwick_data.q[0] = 1.0f;
	madgwick_data.q[1] = 0.0f;
	madgwick_data.q[2] = 0.0f;
	madgwick_data.q[3] = 0.0f;

	lis3mdl.write_reg = lis3mdl_i2c_write;
	lis3mdl.read_reg  = lis3mdl_i2c_read;
	lis3mdl.mdelay    = lis3mdl_delay;
	lis3mdl.handle    = i2c;

	lsm6dso.write_reg = lsm6dso_i2c_write;
	lsm6dso.read_reg = lsm6dso_i2c_read;
	lsm6dso.mdelay = lsm6dso_delay;
	lsm6dso.handle = i2c;
    
    this->is_mag_calibrated = false;
}

bool IMU::init() {
	uint8_t tmp = 0;

	// LSM6DSO
	lsm6dso_device_id_get(&lsm6dso, &tmp);
	if (tmp != LSM6DSO_ID) {
		if (debug) printf("LSM6DSO not found. whoami: 0x%02x\n", tmp);
		return false;
	} else {
		if (debug) printf("LSM6DSO found.");
	}
	tmp = 0;
	lsm6dso_reset_set(&lsm6dso, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(&lsm6dso, &tmp);
	} while (tmp);
	/* Disable I3C interface */
	lsm6dso_i3c_disable_set(&lsm6dso, LSM6DSO_I3C_DISABLE);
	/* Enable Block Data Update */
	lsm6dso_block_data_update_set(&lsm6dso, PROPERTY_ENABLE);
	/* Weight of XL user offset to 2^(-10) g/LSB */
	lsm6dso_xl_offset_weight_set(&lsm6dso, LSM6DSO_LSb_1mg);
	lsm6dso_xl_usr_offset_x_set(&lsm6dso, &accel_offset[0]); // 2's complement
	lsm6dso_xl_usr_offset_y_set(&lsm6dso, &accel_offset[1]);
	lsm6dso_xl_usr_offset_z_set(&lsm6dso, &accel_offset[2]);
	lsm6dso_xl_usr_offset_set(&lsm6dso, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm6dso_xl_data_rate_set(&lsm6dso, LSM6DSO_XL_ODR_417Hz);
	lsm6dso_gy_data_rate_set(&lsm6dso, LSM6DSO_GY_ODR_52Hz);
	/* Set full scale */
#ifdef IMU_ACCEL_16G
	lsm6dso_xl_full_scale_set(&lsm6dso, LSM6DSO_16g);
#elif defined (IMU_ACCEL_8G)
	lsm6dso_xl_full_scale_set(&lsm6dso, LSM6DSO_8g);
#else
	lsm6dso_xl_full_scale_set(&lsm6dso, LSM6DSO_2g);
#endif
	lsm6dso_gy_full_scale_set(&lsm6dso, LSM6DSO_2000dps);
	/* Configure filtering chain(No aux interface). */
	/* Accelerometer - LPF1 + LPF2 path */
	lsm6dso_xl_hp_path_on_out_set(&lsm6dso, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(&lsm6dso, PROPERTY_ENABLE);
    // FREE FALL DETECTION
    lsm6dso_int_notification_set(&lsm6dso, LSM6DSO_ALL_INT_LATCHED);
    lsm6dso_ff_dur_set(&lsm6dso, 0x0a); // 3sample
    lsm6dso_ff_threshold_set(&lsm6dso, LSM6DSO_FF_TSH_312mg);
    lsm6dso_wkup_dur_set(&lsm6dso, 0x04);
    lsm6dso_act_sleep_dur_set(&lsm6dso, 0x04);
    lsm6dso_wkup_threshold_set(&lsm6dso, 0x02);
    lsm6dso_act_mode_set(&lsm6dso, LSM6DSO_XL_12Hz5_GY_PD);
    lsm6dso_pin_int1_route_t int1_route;
    lsm6dso_pin_int1_route_get(&lsm6dso, &int1_route);
    int1_route.free_fall = PROPERTY_ENABLE;
    lsm6dso_pin_int1_route_set(&lsm6dso, int1_route);
    
    // FIFO
#ifdef IMU_FIFO
    lsm6dso_fifo_watermark_set(&lsm6dso, 25); // 20Hz*25times=500ms
    lsm6dso_fifo_xl_batch_set(&lsm6dso, LSM6DSO_XL_BATCHED_AT_52Hz);
    lsm6dso_fifo_gy_batch_set(&lsm6dso, LSM6DSO_GY_BATCHED_AT_52Hz);
    lsm6dso_fifo_mode_set(&lsm6dso, LSM6DSO_STREAM_MODE);
#endif

	// LIS3MDL
	tmp = 0;	
	lis3mdl_device_id_get(&lis3mdl, &tmp);
	if (tmp != LIS3MDL_ID) {
		if (debug) printf("LIS3MDL not found. whoami: 0x%02x\n", tmp);
		return false;
	} else {
		if (debug) printf("LIS3MDL found.");
	}

	tmp = 0;
	lis3mdl_reset_set(&lis3mdl, PROPERTY_ENABLE);
	do {
		lis3mdl_reset_get(&lis3mdl, &tmp);
	} while (tmp);

	/* Enable Block Data Update */
	lis3mdl_block_data_update_set(&lis3mdl, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lis3mdl_data_rate_set(&lis3mdl, LIS3MDL_HP_80Hz);
	/* Set full scale */
	lis3mdl_full_scale_set(&lis3mdl, LIS3MDL_16_GAUSS);
	/* Enable temperature sensor */
	lis3mdl_temperature_meas_set(&lis3mdl, PROPERTY_ENABLE);
	/* Set device in continuous mode */
	lis3mdl_operating_mode_set(&lis3mdl, LIS3MDL_CONTINUOUS_MODE);

	return true;
}

void IMU::calibration(){
    printf("start calibration\n");
    double dx, dy, dz, f;
    double lr=0.000000001;
    int i;
    co[0]=0.0;
    co[1]=0.0;
    co[2]=0.0;
    co[3]=1.0;
    uint8_t reg = 0;
    for(i = 0; i < 80*15; i++){	
        lis3mdl_mag_data_ready_get(&lis3mdl, &reg);
        if (reg) {
            memset(mag_raw, 0x00, 3 * sizeof(int16_t));
            lis3mdl_magnetic_raw_get(&lis3mdl, mag_raw);

            dx= mag_raw[0] - co[0];
            dy= mag_raw[1] - co[1];
            dz= mag_raw[2] - co[2];

            f = dx*dx + dy*dy + dz*dz - co[3]*co[3];
            co[0] = co[0] + 4 * lr * f * dx;
            co[1] = co[1] + 4 * lr * f * dy;
            co[2] = co[2] + 4 * lr * f * dz;
            co[3] = co[3] + 4 * lr * f * co[3];   
        }
        sleep_ms(13); // around 80Hz
    }
    printf("done calibration\n");
}
void IMU::update() {
	uint8_t reg;

#ifdef IMU_FIFO
    uint16_t num = 0;
    uint8_t wmflag = 0;
    lsm6dso_fifo_tag_t reg_tag;
    int16_t dummy[3] = {0};
    /* Read watermark flag */
    lsm6dso_fifo_wtm_flag_get(&lsm6dso, &wmflag);

    if (wmflag > 0) {
      /* Read number of samples in FIFO */
      lsm6dso_fifo_data_level_get(&lsm6dso, &num);

      while (num--) {
        /* Read FIFO tag */
        lsm6dso_fifo_sensor_tag_get(&lsm6dso, &reg_tag);

        switch (reg_tag) {
          case LSM6DSO_XL_NC_TAG:
            memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(&lsm6dso, accel_raw);
#ifdef IMU_ACCEL_16G
		accel_g[0] =
			-lsm6dso_from_fs16_to_mg(accel_raw[0])/1000.0f;
		accel_g[1] =
			lsm6dso_from_fs16_to_mg(accel_raw[1])/1000.0f;
		accel_g[2] =
			lsm6dso_from_fs16_to_mg(accel_raw[2])/1000.0f;
#elif defined(IMU_ACCEL_8G)
		accel_g[0] =
			-lsm6dso_from_fs8_to_mg(accel_raw[0])/1000.0f;
		accel_g[1] =
			lsm6dso_from_fs8_to_mg(accel_raw[1])/1000.0f;
		accel_g[2] =
			lsm6dso_from_fs8_to_mg(accel_raw[2])/1000.0f;
#else
		accel_g[0] =
			-lsm6dso_from_fs2_to_mg(accel_raw[0])/1000.0f;
		accel_g[1] =
			lsm6dso_from_fs2_to_mg(accel_raw[1])/1000.0f;
		accel_g[2] =
			lsm6dso_from_fs2_to_mg(accel_raw[2])/1000.0f;
#endif
//		printf("Acceleration [g]:%4.2f\t%4.2f\t%4.2f\r\n", accel_g[0], accel_g[1], accel_g[2]);
            break;

          case LSM6DSO_GYRO_NC_TAG:
            memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(&lsm6dso, data_raw_angular_rate.u8bit);
		gyro_dps[0] =
			-lsm6dso_from_fs2000_to_mdps(gyro_raw[0])/1000.0f;
		gyro_dps[1] =
			lsm6dso_from_fs2000_to_mdps(gyro_raw[1])/1000.0f;
		gyro_dps[2] =
			lsm6dso_from_fs2000_to_mdps(gyro_raw[2])/1000.0f;
		//		printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
            break;

          default:
            /* Flush unused samples */
            memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(&lsm6dso, dummy);
            break;
        }
      }
    }
  }
#else 
	// LIS3MDL
	/* Read output only if new value is available */
	lis3mdl_mag_data_ready_get(&lis3mdl, &reg);
	if (reg) {
		/* Read magnetic field data */
		memset(mag_raw, 0x00, 3 * sizeof(int16_t));
		lis3mdl_magnetic_raw_get(&lis3mdl, mag_raw);
		mag_mG[1] = -1000 * lis3mdl_from_fs16_to_gauss(
				mag_raw[1]-this->co[1]);
		mag_mG[0] = 1000 * lis3mdl_from_fs16_to_gauss(
				mag_raw[0]-this->co[0]);
		mag_mG[2] = 1000 * lis3mdl_from_fs16_to_gauss(
				mag_raw[2]-this->co[2]);
		//		printf("Magnetic field [mG]:%4.2f %4.2f %4.2f\n", mag_mG[0], mag_mG[1], mag_mG[2]);
	}

	// LSM6DSO
	/* Read output only if new xl value is available */
	lsm6dso_xl_flag_data_ready_get(&lsm6dso, &reg);

	if (reg) {
		/* Read acceleration field data */
		memset(accel_raw, 0x00, 3 * sizeof(int16_t));
		lsm6dso_acceleration_raw_get(&lsm6dso, accel_raw);
#ifdef IMU_ACCEL_16G
		accel_g[0] =
			-lsm6dso_from_fs16_to_mg(accel_raw[0])/1000.0f;
		accel_g[1] =
			lsm6dso_from_fs16_to_mg(accel_raw[1])/1000.0f;
		accel_g[2] =
			lsm6dso_from_fs16_to_mg(accel_raw[2])/1000.0f;
#elif defined(IMU_ACCEL_8G)
		accel_g[0] =
			-lsm6dso_from_fs8_to_mg(accel_raw[0])/1000.0f;
		accel_g[1] =
			lsm6dso_from_fs8_to_mg(accel_raw[1])/1000.0f;
		accel_g[2] =
			lsm6dso_from_fs8_to_mg(accel_raw[2])/1000.0f;
#else
		accel_g[0] =
			-lsm6dso_from_fs2_to_mg(accel_raw[0])/1000.0f;
		accel_g[1] =
			lsm6dso_from_fs2_to_mg(accel_raw[1])/1000.0f;
		accel_g[2] =
			lsm6dso_from_fs2_to_mg(accel_raw[2])/1000.0f;
#endif
//		printf("Acceleration [g]:%4.2f\t%4.2f\t%4.2f\r\n", accel_g[0], accel_g[1], accel_g[2]);
	}

	lsm6dso_gy_flag_data_ready_get(&lsm6dso, &reg);

	if (reg) {
		/* Read angular rate field data */
		memset(gyro_raw, 0x00, 3 * sizeof(int16_t));
		lsm6dso_angular_rate_raw_get(&lsm6dso, gyro_raw);
		gyro_dps[0] =
			-lsm6dso_from_fs2000_to_mdps(gyro_raw[0])/1000.0f;
		gyro_dps[1] =
			lsm6dso_from_fs2000_to_mdps(gyro_raw[1])/1000.0f;
		gyro_dps[2] =
			lsm6dso_from_fs2000_to_mdps(gyro_raw[2])/1000.0f;
		//		printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
	}
#endif // IMU_FIFO
	// 機体の座標系
	if (debug) {
		printf("Acc: %+01.3f %+01.3f %+01.3f, Gyro: %+03.2f %+03.2f %+03.2f, Mag: %+03.2f %+03.2f %+03.2f\n",
				accel_g[0], accel_g[1], accel_g[2],
				gyro_dps[0], gyro_dps[1], gyro_dps[2],
				mag_mG[0], mag_mG[1], mag_mG[2]);
	}
//	MadgwickAHRSupdateIMU(&madgwick_data, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0], accel_g[1], accel_g[2]);
//    float etmp[3];
//    this->getAttEuler(etmp);
//    yawmag = atan((
//        mag_mG[0]*cos(etmp[1])
//        +mag_mG[1]*sin(etmp[1])*sin(etmp[0])
//        +mag_mG[2]*sin(etmp[1])*cos(etmp[0])
//    )/(
//        mag_mG[1]*cos(etmp[0])
//        -mag_mG[2]*sin(etmp[0])
//    ));
	MadgwickAHRSupdate(&madgwick_data, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0], accel_g[1], accel_g[2], mag_mG[0], mag_mG[1], mag_mG[2]);
//	if (debug) printf("%3.2f %3.2f %3.2f %3.2f\n", madgwick_data.q[0], madgwick_data.q[1], madgwick_data.q[2], madgwick_data.q[3]);
}

//void IMU::getMagRaw(float mag[3]) {
//    mag[0] = mag_raw[0];
//    mag[1] = mag_raw[1];
//    mag[2] = mag_raw[2];
//}

void IMU::getAttQuat(float q[4]) {
	q[0] = madgwick_data.q[0];
	q[1] = madgwick_data.q[1];
	q[2] = madgwick_data.q[2];
	q[3] = madgwick_data.q[3];
}

void IMU::getAttEuler(float euler[3]) {
	float quat[4];
	getAttQuat(quat);
	q2e(quat, euler);
}

void IMU::getAccel_g(float accel[3]) {
	accel[0] = accel_g[0];
	accel[1] = accel_g[1];
	accel[2] = accel_g[2];
}

void IMU::getGyro_dps(float gyro[3]) {
	gyro[0] = gyro_dps[0];
	gyro[1] = gyro_dps[1];
	gyro[2] = gyro_dps[2];
}

void IMU::getMag_mG(float mag[3]) {
	mag[0] = mag_mG[0];
	mag[1] = mag_mG[1];
	mag[2] = mag_mG[2];
}

void IMU::setDebugPrint(bool enable) {
	debug = enable;
}

//float IMU::getYawmag() {
//    return ((float)yawmag);
//}

bool IMU::isFreeFallNow() {
    lsm6dso_all_sources_t intr;
    lsm6dso_all_sources_get(&lsm6dso, &intr);
    return (intr.free_fall);
}

// private:
void IMU::q2e(float q[4], float euler[3]) {
	/* 0: roll, 1: pitch, 2: yaw
	   euler[0] = -1.0f * asinf(2.0f * (q[1]) * (q[3]) + 2.0f * (q[0]) * (q[2]));
	   euler[1] = atan2f(2.0f * (q[2]) * (q[3]) - 2.0f * (q[0]) * (q[1]), 2.0f * (q[0]) * (q[0]) + 2.0f * (q[3]) * (q[3]) - 1.0f);
	   euler[2] = atan2f(2.0f * (q[1]) * (q[2]) - 2.0f * (q[0]) * (q[3]), 2.0f * (q[0]) * (q[0]) + 2.0f * (q[1]) * (q[1]) - 1.0f);
	   return; */


	float dqw = madgwick_data.q[0];
	float dqx = madgwick_data.q[1];
	float dqy = madgwick_data.q[2];
	float dqz = madgwick_data.q[3];

	float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;

	// roll (x-axis rotation)
	float t0 = +2.0 * (dqw * dqx + dqy * dqz);
	float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
	float roll = atan2(t0, t1);

	// pitch (y-axis rotation)
	float t2 = +2.0 * (dqw * dqy - dqz * dqx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	float pitch = asin(t2);

	// yaw (z-axis rotation)
	float t3 = +2.0 * (dqw * dqz + dqx * dqy);
	float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
	float yaw = atan2(t3, t4);

	euler[0] = roll;
	euler[1] = pitch;
	euler[2] = yaw;  
}

