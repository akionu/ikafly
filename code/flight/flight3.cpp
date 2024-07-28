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

void nichrom()
{
    gpio_init(pin_nichrome_left);
    gpio_init(pin_nichrome_right);

    gpio_set_dir(pin_nichrome_left, 0);
    gpio_set_dir(pin_nichrome_right, 0);

    gpio_put(pin_nichrome_left, 1);
    gpio_put(pin_nichrome_right, 1);

    vTaskDelay(500);

    gpio_put(pin_nichrome_left, 0);
    gpio_put(pin_nichrome_right, 0);
}

void task_landing(void *landing)
{
    TickType_t llk_landing;
    llk_landing = xTaskGetTickCount();

    press_q = xQueueCreate(32, sizeof(float));

    int p;
    prs.init();
    float alt_m, alt_con[52], alt_av, alt_old;
    int i;

    for (i = 0; i <= 50; i++)
    {
        alt_con[i] = prs.getAltM();
        sleep_ms(100);
    };
    for (i = 10; i <= 50; i++)
    {
        alt_con[51] += alt_con[i];
    };

    alt_av = alt_con[51] / 41;
    alt_old = alt_av;

    // for debug
   // vTaskSuspend(NULL);

   printf("task_landing\n");

    while (1)
    {
        if (xSemaphoreTake(xMutex, (TickType_t)0xffffff) == 1)
        {

            alt_m = prs.getAltM();

            xQueueSend(press_q, &alt_m, 10);
            if (alt_m - alt_av <= 0.01 && alt_av - alt_m <= 0.01)
            {
                // printf("ground");
                if (p >= 10)
                {
                    nichrom();
                    // sprintf("delete_press");
                    vTaskResume(gcontrol_h);
                    landed = 1;
                    vTaskSuspend(NULL);
                }
            }
            else if (alt_m - alt_old >= 0.01)
            {
                // printf("up\n");
            }
            else if (alt_old - alt_m >= 0.01)
            {
                // printf("down\n");
            }
            else
            {
                //  printf("hovering\n");
            };
            alt_old = alt_m;

            if (alt_m - alt_av >= 50)
            {
                p++;
            };

            printf("press");
            xSemaphoreGive(xMutex);
        }
        vTaskDelayUntil(&llk_landing, pdMS_TO_TICKS(200));
    }
}


void task_log(void *log)
{
    TickType_t llk_log;
    llk_log = xTaskGetTickCount();

    int32_t alt_log;
    int32_t log_w[5];
    uint32_t i;

    lfs.init();
    lfs.format();
    lfs.mount();
    lfs.file_open(&file, "ikafly,", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
    printf("task_log\n");

    while (1)
    {
        if (gnss_q != NULL && press_q != NULL && imu_q != NULL)
        {
            if (i % 5 == 0)
            {
                xQueueReset(press_q);
                xQueueReset(gnss_q);
                printf("q_reset");
            }

            if (g_con == 0)
            {

                xQueueReceive(gnss_q, &gdata[1], 10);
                printf("receive");

                log_w[0] = gdata[1].gns[0];
                log_w[1] = gdata[1].gns[1];
                log_w[2] = gdata[1].dis;
                log_w[3] = gdata[1].arg;

                if (landed == 0)
                {
                    xQueueReceive(press_q, &alt_log, 10);
                    log_w[4] = alt_log;
                }
                if (g_con == 1)
                {

                    lfs.file_close(&file);
                    lfs.unmount();
                    vTaskSuspend(gnss_h);
                    vTaskSuspend(imu_h);
                    vTaskResume(ccontrol_h);
                    vTaskSuspend(NULL);
                }
                lfs.file_write(&file, &log_w, sizeof(log_w));
                printf("write log\n");
                i++;
            }
            vTaskDelayUntil(&llk_log, pdMS_TO_TICKS(2000));
            // vTaskDelay(2000);
        }
    }
}

double distance(double longitude, double latitude, double longitude_self, double latitude_self)
{
    double RX = 6378.137;
    double RY = 6356.752;
    double dx = longitude - longitude_self, dy = latitude - latitude_self;
    double mu = (goal_latitude + gga.latitude) / 2.0;
    double E = sqrt(1 - pow(RY / RX, 2.0));
    double W = sqrt(1 - pow(E * sin(mu), 2.0));
    double M = RX * (1 - pow(E, 2.0)) / pow(W, 3.0) / pow(W, 3.0);
    double N = RX / W;
    return sqrt(pow(M * dy, 2.0) + pow(N * dx * cos(mu), 2.0));
}

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
    call++;
    if (call == 294967290)
    {
        call = 0;
    }
}

void task_gnss(void *gnss)
{
    int ch;
    int32_t gga_s[4];

    double arctan;
    double torig[4];
    float yaw;
    double gg[2];

    TickType_t llk_gnss;
    llk_gnss = xTaskGetTickCount();

    gnss_q = xQueueCreate(32, sizeof(gga_s));

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

    uart_init(UART, 9600);
    gpio_set_function(MOSI, GPIO_FUNC_UART);
    gpio_set_function(MISO, GPIO_FUNC_UART);
    uart_set_baudrate(UART, 9600);
    uart_set_hw_flow(UART, false, false);
    uart_set_format(UART, 8, 1, UART_PARITY_NONE);

    uart_set_fifo_enabled(UART, true);
    irq_set_enabled(IRQ, true);
    uart_set_irq_enables(UART, true, false);

    nmeap_init(&nmea, NULL);
    nmeap_addParser(&nmea, "GNGGA", nmeap_gpgga, gpgga_callout, &gga);

    printf("task_gnss");

    while (1)
    {
        call_a = call;
        ch = uart_getc(UART);
        nmeap_parse(&nmea, ch);

        if (call != call_a)
        {


            if (g_con == 1)
            {
                gg[0] = gga.latitude;
                gg[1] = gga.longitude;
                lfs.mount();
                lfs.file_open(&file, "gg", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
                lfs.file_write(&file, &gg, sizeof(gg));
                lfs.file_close(&file);
                lfs.unmount();
                vTaskSuspend(ccontrol_h);
            }
            

            xQueueReceive(imu_q, &yaw, 10);
            arctan = atan2(goal_longitude - gga.longitude, goal_latitude - gga.latitude);
            torig[0] = cos(arctan);
            torig[1] = sin(arctan);
            torig[2] = cos(yaw);
            torig[3] = sin(yaw);

            gdata[0].gns[0] = gga.latitude * 10000000;
            gdata[0].gns[1] = gga.longitude * 10000000;
            gdata[0].dis = distance(goal_longitude, goal_latitude, gga.longitude, gga.latitude);
            gdata[0].arg = atan2(-torig[3] * torig[0] + torig[2] * torig[1], torig[2] * torig[0] + torig[3] * torig[1]);

            xQueueSend(gnss_q, &gdata[0], 10);
            vTaskDelayUntil(&llk_gnss, pdMS_TO_TICKS(150));
        }
    }
}

void task_imu(void *imu_t)
{
    TickType_t llk_imu;
    llk_imu = xTaskGetTickCount();
    imu_q = xQueueCreate(512, sizeof(float));
    imu.init();
    imu.setDebugPrint(false);
    // imu.calibration();



    float euler[3];
    printf("task_imu");
    while (1)
    {
        if (xSemaphoreTake(xMutex, (TickType_t)0xffffff) == 1)
        {
            imu.update();
            imu.getAttEuler(euler);
            xQueueSend(imu_q, &euler[2], 10);
            printf("imu");
            xSemaphoreGive(xMutex);
        }
        vTaskDelayUntil(&llk_imu, pdMS_TO_TICKS(20));
        // vTaskDelay(20);
    }
}

void task_gcontrol(void *gcontrol)
{
    vTaskSuspend(NULL);

    printf("g_motor");
    float p;
    int j = 0;
    float dis_ot = 100;
    float dis_ot_g;

    double dis_m;
    double arg_m;

    uint8_t l = 0;

    motor.init(pin_left_begin, pin_right_begin);

    TickType_t llk_gcontrol;
    llk_gcontrol = xTaskGetTickCount();

    while (1)
    {

        if (gnss_q != NULL && press_q != NULL && imu_q != NULL)
        {

            printf("move\n");

            // calculate argument between ikafly and goal

            /*if (latitude_ot != 0)
            {
                dis_ot = distance(longitude_ot, latitude_ot, gga.longitude, gga.latitude);
                dis_ot_g = distance(longitude_ot, latitude_ot, gga.longitude, gga.latitude);
            }*/

            xQueueReceive(gnss_q, &gdata[2], 10);

            dis_m = gdata[2].dis;
            arg_m = gdata[2].arg;

            if (dis_m > 0)
            {

                /* if (j == 0)
             {
                 motor.setDirForward(1, 1);
                 j++;
             }

             p = arg_m * 39.15;

             if (p >= 123)
             {
                 p = 123;
             }
             else if (p <= -123)
             {
                 p = -123;
             }

             if (dis_m < 2)
             {
                 vTaskSuspend(NULL);
             }
             else if (dis_ot >= 2)
             {

                 motor.forward(900 + p, 900 - p);
             }
             else if (dis_m - dis_ot_g <= 0)
             {

                 motor.forward(900 + p, 900 - p);
             }
             else
             {

                 motor.stop();
                 printf("stop");
                 vTaskSuspend(NULL);
                 //vTaskResume(cam_motor_control_h);
             } */

                if (j == 0)
                {
                    motor.setDirForward(1, 1);
                    j++;
                }

                p = arg_m * 39.15;

                if (p >= 123)
                {
                    p = 123;
                }
                else if (p <= -123)
                {
                    p = -123;
                }

                motor.forward(900 + p, 900 - p);
                if (dis_m <= 2.0)
                {
                    l++;
                    motor.stop();
                    if (l > 5)
                    {
                        g_con = 1;
                        vTaskResume(ccontrol_h);
                        vTaskSuspend(gnss_h);
                        vTaskSuspend(NULL);
                    }
                }
            }
        }
        printf("papa\n");
        vTaskDelayUntil(&llk_gcontrol, pdMS_TO_TICKS(200));
    }
}

static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect);
void count_vert(uint8_t vert[5], uint8_t area[15]);

static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect)
{
    // ref:https://qiita.com/yosihisa/items/c326e59ca5d65f35a181
    uint8_t *src;
    uint16_t x, y; //, bws;
    uint8_t r, g, b;
    double h = 0, s, v;
    uint32_t bitshift = 0;

    //    printf("\n");
    src = (uint8_t *)bitmap;
    //    bws = N_BPP * (rect->right - rect->left + 1);
    for (y = 0; y < (rect->bottom - rect->top + 1); y++)
    {
        for (x = 0; x < (rect->right - rect->left + 1); x += 1)
        {
            //            printf("y: %d\n", y);
            r = *(src + 3 * (y * (rect->right - rect->left + 1) + x));
            g = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 1);
            b = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 2);
            //            printf("(%03d,%03d,%03d),", r,g,b);
            double MAX = max((max(r, g)), b);
            double MIN = min((min(r, g)), b);
            v = MAX / 256 * 100;

            if (MAX == MIN)
            {
                h = 0;
                s = 0;
            }
            else
            {
                if (MAX == r)
                    h = 60.0 * (g - b) / (MAX - MIN) + 0;
                else if (MAX == g)
                    h = 60.0 * (b - r) / (MAX - MIN) + 120.0;
                else if (MAX == b)
                    h = 60.0 * (r - g) / (MAX - MIN) + 240.0;

                if (h > 360.0)
                    h = h - 360.0;
                else if (h < 0)
                    h = h + 360.0;
                s = (MAX - MIN) / MAX * 100.0;
            }

            if (h > 360.0)
                h -= 360;
            // 赤色の判定
            if ((h >= H_MIN_1 && h <= H_MAX_1) || (h >= H_MIN_2 && h <= H_MAX_2))
            {
                if ((s >= S_MIN && s <= S_MAX) && (v >= V_MIN && v <= V_MAX))
                {
                    // printf("red!: (rx,ry)=(%d,%d)\n", rect->left+x, rect->top+y);
                    bitshift = (rect->left + x);
                    red_area[rect->top + y] |= (1 << bitshift);
                }
            }
        }
    }

    return 1; // Continue to decompress
}

void count_vert(uint8_t vert[5], uint32_t area[15])
{
    memset(vert, 0, 5);
    uint8_t bit;
    for (uint8_t y = 0; y < 15; y++)
    {
        for (int8_t x = 20 - 1; x >= 0; x--)
        {
            bit = (area[y] >> x) & 1;
            if (bit == 1)
            {
                if ((x >= 0) && (x < 4))
                    vert[0]++;
                else if (x < 8)
                    vert[1]++;
                else if (x < 12)
                    vert[2]++;
                else if (x < 16)
                    vert[3]++;
                else
                    vert[4]++;
            }
        }
    }
}

void task_ccontrol(void *ccontrol)
{
    vTaskSuspend(NULL);

    cam.init();
    cam.enableJpeg();

    // for debug
   // motor.init(pin_left_begin, pin_right_begin);

    TickType_t llk_ccontrol;
    llk_ccontrol = xTaskGetTickCount();

    uint32_t cnt = 0;
    uint8_t vert[5] = {0};

    while (1)
    {
        vTaskSuspend(gnss_h);
        if (xSemaphoreTake(xMutex, (TickType_t)0xffffff) == 1)
        {
            cam.capture();
            while (!cam.isCaptureFinished())
                tight_loop_contents();
            uint32_t size = cam.getJpegSize();

            memset(red_area, 0, sizeof(red_area));

            JRESULT res = tjpg.prepare(cam.image_buf, size);
            if (res == JDR_OK)
            {
                res = tjpg.decomp(my_out_func, 3);

                if (res == JDR_OK)
                {
                    uint8_t bit;
                    for (uint8_t y = 0; y < 15; y++)
                    {
                        for (int8_t i = 20 - 1; i >= 0; i--)
                        {
                            bit = (red_area[y] >> i) & 1;
                        }
                    }

                    count_vert(vert, red_area);

                    if (vert[0] > 50 && vert[1] > 50 && vert[2] > 50 && vert[3] > 50 && vert[4] > 50)
                    {
                        goal = 1;
                        printf("goal");
                        xSemaphoreGive(xMutex);
                        vTaskSuspend(NULL);
                    }

                    int8_t dire = 0;
                    for (int8_t q = 0; q < 3; q++)
                    {
                        if (vert[q + 1] >= vert[q])
                        {
                            dire = q + 1;
                        }
                    }

                    printf("ccontrol");

                    switch (dire)
                    {

                    case 0:

                        motor.forward(800, 1023);

                        break;

                    case 1:

                        motor.forward(900, 1023);

                        break;

                    case 2:

                        motor.forward(1023, 1023);

                        break;

                    case 3:

                        motor.forward(900, 1023);

                        break;

                    case 4:

                        motor.forward(800, 1023);

                        break;

                    default:

                        motor.forward(0, 1023);
                    }
                }
            }
            xSemaphoreGive(xMutex);
        }
        // vTaskDelayUntil(&llk_ccontrol, pdMS_TO_TICKS(200));
        vTaskDelay(200);
    }
}

void task_send(void *send)
{
    vTaskSuspend(NULL);
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C, 400 * 1000);
    gpio_set_function(pin_i2c1_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_i2c1_scl, GPIO_FUNC_I2C);

    xMutex = xSemaphoreCreateMutex();

    xTaskCreate(task_log, "task_log", 1024, NULL, 9, &log_h);
    xTaskCreate(task_landing, "task_landing", 1024, NULL, 3, &landing_h);
    xTaskCreate(task_gnss, "task_gnss", 1024, NULL, 1, &gnss_h);
    xTaskCreate(task_imu, "task_imu", 1024, NULL, 2, &imu_h);
    xTaskCreate(task_gcontrol, "task_gcontrol", 1024, NULL, 4, &gcontrol_h);
    xTaskCreate(task_ccontrol, "task_ccontrol", 1024, NULL, 5, &ccontrol_h);
    // xTaskCreate(task_send,"task_send",1024,NULL,5,&send_h);
    vTaskStartScheduler();
    while (1)
    {
    };
}