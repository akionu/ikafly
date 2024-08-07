#pragma once
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define IMU_ACCEL_16G

extern "C" {
	#include "./lis3mdl/src/lis3mdl.h"
	#include "./lsm6dso/src/lsm6dso.h"
	#include "./MadgwickAHRS/MadgwickAHRS.h"
}

class IMU 
{
	public:
		IMU(i2c_inst_t* i2c);
		bool init(); // return true if success, false if fail
		void calibration();//calibration
		void update(); // call this 50ms each for get attitude
		void getAttQuat(float q[4]);
		void getAttEuler(float euler[3]); // roll, pitch, yaw
		float getAttEulerRoll();

		void getAccel_g(float accel[3]); // x,y,z
		void getGyro_dps(float gyro[3]); // x,y,z
		void getMag_mG(float mag[3]); // x,y,z

		void setAccelOffset(uint8_t offset[3]); // x,y,z, [mg]
		
		void setDebugPrint(bool enable);
        bool isFreeFallNow();
		double co[4]; // coefficient of mag
	private:
		void q2e(float q[4], float euler[3]);
		bool debug = false;
		madgwick_ahrs_t madgwick_data;
		float accel_g[3];
		float gyro_dps[3];
		float mag_mG[3];
		bool is_mag_calibrated = false;
		int16_t accel_raw[3];
		int16_t gyro_raw[3];
		int16_t mag_raw[3];
		uint8_t accel_offset[3] = {0};
		stmdev_ctx_t lsm6dso;
		stmdev_ctx_t lis3mdl;
};
