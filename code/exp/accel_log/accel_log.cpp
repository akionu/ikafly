
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"
#include "littlefs.hpp"
#include "imu.h"

#define I2C i2c1
#define SDA 14
#define SCL 15

#define RAD2DEG 57.2958

IMU imu(I2C);
LFS lfs;
lfs_file_t file;

void task_log(void *p) {
    bool st = imu.init();
    if (st) printf("IMU init success");
    else printf("IMU init fails");

    int err = lfs.init();
    lfs.format();
    err = lfs.mount();
    if (err) {
        lfs.format();
        lfs.mount();
        printf("err lfs but ok\n");
    }
    lfs.file_open(&file, "accel", LFS_O_RDWR | LFS_O_CREAT);
    //    lfs.file_rewind(&file);

    float accel[3];
    //    TickType_t lastwake = xTaskGetTickCount();
    uint8_t i = 0;
    uint32_t now_ms = 0;
    while (1) {
        now_ms = time_us_32()/1000;
        imu.update();
        imu.getAccel_g(accel);
        //printf("%d ", time_us_32()/1000);
        //		printf("%3.2f %3.2f %3.2f\n", accel[0], accel[1], accel[2]);
//        lfs.file_write(&file, &now_ms, sizeof(now_ms));
        lfs.file_write(&file, accel, sizeof(accel));
        //        if (i == 20) {
        //           printf("s\n");
        //           i = 0;
        lfs.file_sync(&file);
        //      } else {
        //         i++;
        //    }

        // 12Byte per 50 ms(20Hz)
        // 14.4KB per 1 min
        vTaskDelay(10/portTICK_PERIOD_MS);
        //        vTaskDelayUntil(&lastwake, 200 / portTICK_PERIOD_MS); 
    }
}

int main(void) {
    stdio_usb_init();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);

    // I2C init
    i2c_init(I2C, 400 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);

    imu.setDebugPrint(false);

    xTaskCreate(task_log, "task_log", 512, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1) {}
}
