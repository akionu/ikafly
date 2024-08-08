#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../lib/imu/src/imu.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <math.h>
#include "../../lib/ikafly_pin.h"


#define I2C i2c1

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;
#define RAD2DEG 57.2958


IMU imu(I2C);

int main(void){

    stdio_init_all();
	sleep_ms(2000);
	i2c_init(I2C, 400*1000);

	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
    imu.init();
	imu.setDebugPrint(false);
    imu.calibration();
	float euler[3];

while(1){
        
        imu.update();
		imu.getAttEuler(euler);
        printf("%+03.2f\n",euler[2]*RAD2DEG);
		sleep_ms(2);
		}
}

