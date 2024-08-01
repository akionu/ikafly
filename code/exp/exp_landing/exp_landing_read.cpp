#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
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

#define I2C i2c1
#define SDA 14
#define SCL 15

#define RAD2DEG 57.2958

LFS lfs;
lfs_file_t file;

float buf[25*1024] = {0}; // 60kB

void task_log(void *p) {
//	bool st = imu.init();
//	if (st) printf("IMU init success");
//	else printf("IMU init fails");

    int err = lfs.init();
    err = lfs.mount();
    if (err) {
        lfs.format();
        lfs.mount();
        printf("err lfs but ok\n");
    }
    lfs.file_open(&file, "accel", LFS_O_RDONLY);
    lfs.file_read(&file, buf, 25*1024);
    
    for (uint32_t i = 0; buf[i] != 0; i+=4) {
//        printf("%d,%3.3f,%3.3f,%3.3f\n", (uint32_t)buf[i], buf[i+1], buf[i+2], buf[i+3]);
        printf("%3.3f,%3.3f,%3.3f\n", buf[i], buf[i+1], buf[i+2]);
    }

    while(1) {}
}

int main(void) {
    stdio_usb_init();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);


    xTaskCreate(task_log, "task_log", 512, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1) {}
}
