#include <cmath>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/platform.h"
#include "pico/error.h"
#include "pico/flash.h"

#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

#include "uprs.h"
#include "littlefs.hpp"
#include "imu.h"
#include "../lib/cam/src/cam.h"
#include "../lib/tjpg/src/tjpg.hpp"
#include "../lib/motor/src/umotor.h"
#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../lib/ikafly_pin.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define I2C i2c1
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define RAD2DEG 57.2958
#define goal_latitude 34.801665
#define goal_longitude 135.771090

#define XCLK 0
#define Y2_PIO_BASE 1
#define PIO pio0
#define PIO_SM 0
#define DMA_CH 0
#define H_MIN_1 0 // 固定
#define H_MAX_1 60
#define H_MIN_2 300
#define H_MAX_2 360 // 固定
// Satuation
#define S_MIN 10
#define S_MAX 100
// Value
#define V_MIN 15
#define V_MAX 90

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

Press prs(I2C, 0x77);
IMU imu(I2C);
LFS lfs;
lfs_file_t file;
nmeap_context_t nmea;
nmeap_gga_t gga;
Motor motor;
Cam cam(I2C, XCLK, Y2_PIO_BASE, PIO, PIO_SM, DMA_CH);
TJPGD tjpg;

SemaphoreHandle_t xMutex = NULL;

QueueHandle_t press_q = NULL;
QueueHandle_t gnss_q = NULL;
QueueHandle_t imu_q = NULL;

TaskHandle_t landing_h;
TaskHandle_t gnss_h;
TaskHandle_t log_h;
TaskHandle_t imu_h;
TaskHandle_t gcontrol_h;
TaskHandle_t ccontrol_h;
TaskHandle_t send_h;
TaskHandle_t receive_h;

// uint8_t landed = 0;
// for debug
uint8_t landed = 0;

uint8_t goal = 0;
uint8_t g_con = 0;

uint32_t call = 0;
uint32_t call_a = 0;
uint32_t red_area[15] = {0};

typedef struct
{
    uint32_t gns[2];
    uint8_t dis;
    uint8_t arg;
} gnss_data;

gnss_data gdata[4];
