#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../lib/imu/src/imu.h"

#include "FreeRTOSConfig.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/task.h"


#define I2C i2c1

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;
#define RAD2DEG 57.2958

IMU imu(I2C);

void task_imu(void *){
	float euler_1;
	float euler_2;
	float euler_i;
	vTaskDelay(2000);
	imu.setDebugPrint(false);

	bool st = imu.init();
	if (st) printf("IMU init success");
	else printf("IMU init fails");
    imu.calibration();
	float mag[3],mag_s[64],mag_av_1,mag_av_2,euler[3];
	float mag_sum_1=0;
	float mag_sum_2=0;
	while (1) {
		int k;
		for(k=0;k<=20;k++){
		imu.update(); // should call this 50ms each(20Hz), refer MadgwickAHRS.c and sensor output frequency in init() in imu.cpp
		//imu.getAccel_mg(accel);
		//printf("%3.2f %3.2f %3.2f\n", accel[0], accel[1], accel[2]);
		//printf("%+03.2f %+03.2f %+03.2f\n", euler[0]*RAD2DEG, euler[1]*RAD2DEG, euler[2]*RAD2DEG);
	    //printf("%f,%f,%f\n",mag[0],mag[1],mag[2]);

	

		imu.getAttEuler(euler);
		imu.getMag_mG(mag);



		mag_s[3*k]=mag[0];
		mag_s[3*k+1]=mag[1];
		mag_s[3*k+2]=mag[2];


		}

		for(k=0;k<=2;k++){
			if(euler[k]<0){
				euler[k]=360+euler[k];
			}
		}

		for(k=1;k<=19;k++){
			mag_sum_1=mag_sum_1+mag_s[3*k]-mag_s[3*(k-1)];
		}

		for(k=1;k<=19;k++){
			mag_sum_2=mag_sum_2+mag_s[3*k+1]-mag_s[3*(k-1)+1];
		}

		mag_av_1=mag_sum_1/19;
		mag_av_2=mag_sum_2/19;

		printf("mag_av_1,%f\n",mag_av_1);


	euler_1=atan(mag[0]/sqrt((mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2])*cos(euler[1])*cos(euler[1])-mag[0]*mag[0]));
	euler_2=atan(mag[0]/-sqrt((mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2])*cos(euler[1])*cos(euler[1])-mag[0]*mag[0]));

	printf("euler_1,%f\n",euler_1);
	printf("euler_2,%f\n",euler_2);

	if(euler_1=euler_2){
		euler_i=euler_1-euler[2];
	}else if(mag_av_1/mag_av_2>0){
		if(euler_1<=90 && euler_1>=0){
			euler_i=euler_1-euler[2];

		}else if(euler_1>=180 && euler_1<=270 ){
			euler_i=euler_1-euler[2];

		}else if(euler_2<=90 && euler_2>=0 ){
			euler_i=euler_2-euler[2];

		}else if(euler_2>=180 && euler_2<=270 ){
			euler_i=euler_2-euler[2];
	}
	}else if(mag_av_1/mag_av_2<0){
		if(euler_1<=180 && euler_1>=90){
			euler_i=euler_1-euler[2];

		}else if(euler_1>=270 && euler_1<=360 ){
			euler_i=euler_1-euler[2];

		}else if(euler_2<=180 && euler_2>=90 ){
			euler_i=euler_2-euler[2];
		}else if(euler_2>=270 && euler_2<=360 ){
			euler_i=euler_2-euler[2];
	}
	}
	printf("%f\n",euler_i);
}
}

int main(void)
{
	stdio_init_all();
	i2c_init(I2C, 400*1000);
	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
	xTaskCreate(task_imu,"task_imu",256,NULL,1,NULL);
    vTaskStartScheduler();
    while(1){};
    
}
