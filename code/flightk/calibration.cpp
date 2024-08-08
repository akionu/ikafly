#include <stdio.h>
#include <math.h>
#include <stdarg.h>

#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/pio.h"

#include "littlefs.hpp"
#include "imu.h"
#include "../lib/ikafly_pin.h"

#include "ulog.h"
#include "angle.h"


extern "C" {
#include "util.h"
}
#define CONST_180_DIVIDED_BY_PI 57.2957795130823
#define RAD2DEG 57.2958
Log logging(0);
IMU imu(i2c1);

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(pin_i2c1_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_i2c1_scl, GPIO_FUNC_I2C);

    // imu
    bool st = imu.init();
    printf("IMU init %s\n", st?"ok":"fail");

    // led
    gpio_init(pin_led);
    gpio_put(pin_led, 0);

    // logging
    bool ret = logging.init();
    printf("logging: %s\n", ret?"ok":"fail");
    
    sleep_ms(1000);

    // imu calibration
    printf("always calibrate mode. calibrating...\n");
    imu.calibration();
    logging.storeCalibData(imu.co);
    logging.readCalibData(imu.co);
    printf("co: %f %f %f %f\n", imu.co[0], imu.co[1], imu.co[2], imu.co[3]);
    printf("calibration data saved.\n");

    float euler[3];
    while (1) {
        imu.update();
		imu.getAttEuler(euler);
		printf("%+03.2f %+03.2f %+03.2f\n", euler[0]*RAD2DEG, euler[1]*RAD2DEG, euler[2]*RAD2DEG);
		sleep_ms(19);
    }
}
