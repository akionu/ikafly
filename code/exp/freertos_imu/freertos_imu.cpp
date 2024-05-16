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
#define SDA 14
#define SCL 15

#define RAD2DEG 57.2958

IMU imu(I2C);

void task_imu(void *){
	stdio_init_all();
	vTaskDelay(2000);
	printf("This is imu_euler\n");

	// I2C init
	i2c_init(I2C, 400 * 1000);
	gpio_set_function(SDA, GPIO_FUNC_I2C);
	gpio_set_function(SCL, GPIO_FUNC_I2C);

	imu.setDebugPrint(false);

	bool st = imu.init();
	if (st) printf("IMU init success");
	else printf("IMU init fails");

	float euler[3];//, accel[3], gyro[3], mag[3];
	printf("roll pitch yaw\n");
	while (1) {
		imu.update(); // should call this 50ms each(20Hz), refer MadgwickAHRS.c and sensor output frequency in init() in imu.cpp
		//imu.getAccel_mg(accel);
		//printf("%3.2f %3.2f %3.2f\n", accel[0], accel[1], accel[2]);
		imu.getAttEuler(euler);
		printf("%+03.2f %+03.2f %+03.2f\n", euler[0]*RAD2DEG, euler[1]*RAD2DEG, euler[2]*RAD2DEG);
		vTaskDelay(49);
	}
}

 int main(void){
	stdio_init_all();
	xTaskCreate(task_imu,"task_imu",256,NULL,1,NULL);
    vTaskStartScheduler();
	while(1){};
 }