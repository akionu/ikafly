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
#include "uprs.h"

#define I2C i2c1
#define SDA 14
#define SCL 15

#define RAD2DEG 57.2958
#define SEC2CNT 2

IMU imu(I2C);
Press prs(I2C, 0x77);
LFS lfs;
lfs_file_t file;


SemaphoreHandle_t xMutex;

enum {
    MODE_LANDING = 1,
    MODE_NICHROME,
};

int islanding(float* alt_change, bool* isDetectRise, bool* isDetectFall);
bool lock() {
    return (xSemaphoreTake(xMutex, (TickType_t)1000) == pdTRUE);
}
bool unlock() {
    return (xSemaphoreGive(xMutex) == pdTRUE);
}
int islanding(float* alt_change, bool* isDetectRise, bool* isDetectFall) {
    for (int8_t i = 1; i < 10; i++) {
        alt_change[i-1] = alt_change[i];
    }
    if (lock()) {
        alt_change[9] = prs.getAltM();
        unlock();
    }
    printf("alt: %f\n", alt_change[9]);
    printf("rise: %d, fall: %d\n", *isDetectRise, *isDetectFall);
    for (int i = 0; i < 10; i++) printf("%3.2f ", alt_change[i]);
    printf("\n");

    if (!isDetectRise) {
        if (alt_change[0] >= 3000) {
            return MODE_LANDING;
        }
        int8_t cnt = 0;
        for (int8_t i = 1; i < 10; i++) {
            // さっきより高度が高い=より上にいる
            if ((alt_change[i] - alt_change[i-1]) > 0.05) cnt++;
            //printf("isDetectRise: %d\n", cnt);
        }
        if (cnt > 4) {
            *isDetectRise = true;
        }
    }
    if (isDetectRise && !isDetectFall) {
        if (alt_change[0] >= 1000) return MODE_LANDING;
        int8_t cnt = 0;
        for (int8_t i = 1; i < 10; i++) {
            // さっきより高度が低い=より下にいる（地面に近い）
            if ((alt_change[i-1] - alt_change[i]) > 0.05) cnt++;
        }
        if (cnt > 3) {
            *isDetectFall = true;
            return MODE_NICHROME;
        }
    } 
//    if (isDetectRise && isDetectFall) {
//
//        // 分散
//        double avg = 0, s = 0;
//        for (int8_t i = 0; i < 10; i++) {
//            avg += alt_change[i];
//        }
//        avg /= 10;
//        for (int8_t i = 0; i < 10; i++) {
//            s += (alt_change[i]-avg)*(alt_change[i]-avg);
//        }
//        s /= 10;
//        printf("alt: %f, s: %f\n", alt_change[9], s);
//        if (s < 0.06) {
//            return MODE_NICHROME;
//        }
//    } else {
//        //landingCnt = 0;
//        // 300s
//        if (landingCnt > (300*SEC2CNT)) {
//            if (landingCnt > (500*SEC2CNT)) {
//                return MODE_NICHROME;
//            }
//        } else {
//            landingCnt++;
//        }
//    }
    return MODE_LANDING;
}
void task_log(void *p) {
    static float accel[1000];
    float alt_change[10] = {0};
    bool isDetectRise = false;
    bool isDetectFall = false;

    // I2C init
    i2c_init(I2C, 400 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);

	if (lock()) {
        prs.init();
        unlock();
    }

	if (lock()) {
        bool st = imu.init();
        if (st) printf("IMU init success");
        else printf("IMU init fails");
    
        imu.setDebugPrint(false);
        unlock();
    }


    int err = lfs.init();
//    lfs.format();
//    err = lfs.mount();
//    if (err) {
//        lfs.format();
//        lfs.mount();
//        printf("err lfs but ok\n");
//    }
//    lfs.file_open(&file, "accel", LFS_O_RDWR | LFS_O_CREAT);
//    lfs.file_rewind(&file);

    TickType_t lastwake = xTaskGetTickCount();
    uint32_t now_ms = 0;
    int8_t res = 0;
    for (uint32_t i = 0; ; i+=3) {
        now_ms = time_us_32()/1000;
        printf("%d, %d: ", now_ms, i);
        if (lock()) {
            imu.update();
            imu.getAccel_g(&accel[i]);
            unlock();
        }
        if (MODE_NICHROME == islanding(alt_change, &isDetectRise, &isDetectFall)) {
            printf("nichrome\n");
        }
        if (i >= 990) break;
        vTaskDelayUntil(&lastwake, 500/portTICK_PERIOD_MS);
        //vTaskDelay(500/portTICK_PERIOD_MS);
    }

    lfs.file_write(&file, accel, sizeof(accel));
    lfs.file_sync(&file);

    while (1) {
        vTaskDelay(1000);
    }
}


int main(void) {
    stdio_usb_init();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);


	xMutex = xSemaphoreCreateMutex();

    xTaskCreate(task_log, "task_log", 512, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1) {}
}
