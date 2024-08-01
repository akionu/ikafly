
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

//#define T_MS 10 // 100Hz
#define T_MS 20 // 50Hz
#define FREQ (1000/T_MS)
#define DUR_S 15
//#define DUR_S 60

IMU imu(I2C);
LFS lfs;
lfs_file_t file;

float accel_buf[3*FREQ*DUR_S] = {0};

void task_log(void *p) {
    bool st = imu.init();
    if (st) printf("IMU init success\n");
    else printf("IMU init fails\n");

    int err = lfs.init(); printf("init: %d\n", err);
    err = lfs.format();printf("format: %d\n", err);
    err = lfs.mount();printf("mount: %d\n", err);
    if (err) {
        lfs.format();
        lfs.mount();
        printf("err lfs but ok\n");
    }
    err = lfs.file_open(&file, "accel", LFS_O_RDWR | LFS_O_CREAT);printf("open: %d\n", err);

    err = lfs.file_rewind(&file); printf("rewind: %d\n", err);

    TickType_t lastwake = xTaskGetTickCount();
    uint32_t now_ms = 0;
    for (int32_t i = 0; i < 3*(FREQ)*DUR_S; i+=3){
        now_ms = time_us_32()/1000;
        imu.update();
        imu.getAccel_g(&accel_buf[i]);
//      iji  memcpy(&accel_buf[i], accel, sizeof(float)*3);
        printf("%d %d: %3.3f %3.3f %3.3f\n", now_ms, i, accel_buf[i], accel_buf[i+1], accel_buf[i+2]);
        // 12Byte per 50 ms(20Hz)
        // 14.4KB per 1 min
//        vTaskDelay(50/portTICK_PERIOD_MS);
        vTaskDelayUntil(&lastwake, T_MS / portTICK_PERIOD_MS); 
    }

    err = lfs.file_write(&file, accel_buf, sizeof(accel_buf));printf("write: %d\n", err);
    err = lfs.file_sync(&file);printf("sync: %d\n", err);
    err = lfs.file_close(&file);printf("close: %d\n", err);
    printf("written\n");

    // read back
//    memset(accel_buf, 0, sizeof(accel_buf));
//    err = lfs.file_open(&file, "accel", LFS_O_RDONLY);printf("open: %d\n", err);
//    int32_t n = lfs.file_read(&file, accel_buf, sizeof(accel_buf));printf("read: %d\n", err);
//    for (int32_t i = 0; i < (n/int32_t(sizeof(float))); i+=3) {
//        printf("%3.3f,%3.3f,%3.3f\n", accel_buf[i], accel_buf[i+1], accel_buf[i+2]);
//    }

    err = lfs.unmount(); printf("unmount: %d\n", err);
    gpio_put(29, 1);

    while (1) {
        vTaskDelay(1000);
    }
}

int main(void) {
    stdio_usb_init();
//    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);
    gpio_init(29);
    gpio_set_dir(29, GPIO_OUT);
    gpio_put(29,0);

    // I2C init
    i2c_init(I2C, 400 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);

    imu.setDebugPrint(false);

    xTaskCreate(task_log, "task_log", 512, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1) {}
}
