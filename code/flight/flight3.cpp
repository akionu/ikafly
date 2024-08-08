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
#include "rf.h"
#include "cam.h"
#include "tjpg.hpp"
#include "umotor.h"
#include "nmeap.h"
#include "../lib/ikafly_pin.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define I2C i2c1
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define RAD2DEG 57.2958
#define thrty 0.523599
#define goal_latitude 34.801609
#define goal_longitude 135.770979
#define EARTH_RAD 6378137
#define rp 0.01745329251

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
Radio radio(24, 22);

SemaphoreHandle_t xMutex = NULL;
EventGroupHandle_t gcs_event = NULL;
EventGroupHandle_t gi_enent = NULL;

QueueHandle_t press_q = NULL;
QueueHandle_t gnss_q = NULL;
QueueHandle_t imu_q = NULL;
QueueHandle_t receive_q = NULL;
QueueHandle_t stuck_q = NULL;

TaskHandle_t landing_h;
TaskHandle_t gnss_h;
TaskHandle_t log_h;
TaskHandle_t imu_h;
TaskHandle_t gcontrol_h;
TaskHandle_t ccontrol_h;
TaskHandle_t send_h;
TaskHandle_t receive_h;
TaskHandle_t stuck_h;

// uint8_t landed = 0;
// for debug
uint8_t landed = 0;
uint8_t goal = 0;
uint8_t g_con = 0;
uint8_t gbit = (1 << 0);
uint8_t cbit = (1 << 0);
uint8_t ibit = (1 << 0);
uint8_t result = NULL;
uint8_t result_s = NULL;

uint32_t call = 0;
uint32_t call_a = 0;
uint32_t red_area[15] = {0};

typedef struct
{
    uint32_t gns[2];
    uint16_t dis;
    uint8_t arg;
} gnss_data;

gnss_data gdata[6];

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

  //  printf("task_landing");
    vTaskSuspend(NULL);

    // for debug
    // vTaskSuspend(NULL);

    TickType_t llk_landing;
    llk_landing = xTaskGetTickCount();

    int j = 0;

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

                    landed = 1;
                    xSemaphoreGive(xMutex);
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

            // for debug
            j++;
            if (j == 30)
            {
                landed = 1;
                //  nichrom();
              //  printf("delete_press");
                xSemaphoreGive(xMutex);
                vTaskSuspend(NULL);
            }

      //      printf("press");
            xSemaphoreGive(xMutex);
        }
        vTaskDelayUntil(&llk_landing, pdMS_TO_TICKS(200));
    }
}

void task_log(void *log)
{

    int32_t alt_log;
    int32_t log_w[5];

    uint32_t i;

    lfs.init();
    lfs.format();

    lfs.mount();
    lfs.file_open(&file, "ikafly,", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);

  //  printf("task_log\n");

    // for debug
    vTaskSuspend(NULL);

    TickType_t llk_log;
    llk_log = xTaskGetTickCount();

    int l = 0;

    while (1)
    {
        if (gnss_q != NULL && press_q != NULL && imu_q != NULL)
        {
            if (i % 10 == 0)
            {
                xQueueReset(press_q);
                xQueueReset(gnss_q);
                xQueueReset(stuck_q);
                xQueueReset(receive_q);

                //printf("q_reset");
            }

            if (xQueueReceive(gnss_q, &gdata[1], 10) == 1)
            {
           //     printf("receive");
                log_w[0] = gdata[1].gns[0];
                log_w[1] = gdata[1].gns[1];
                log_w[2] = gdata[1].dis;
                log_w[3] = gdata[1].arg;
            }

            if (landed == 0)
            {
                printf("landing");
                if (xQueueReceive(press_q, &alt_log, 10) == 1)
                {
                    log_w[4] = alt_log;
                }
            }
            else if (l == 0)
            {
                vTaskResume(gcontrol_h);
                vTaskDelay(10);
                vTaskResume(receive_h);
                vTaskDelay(10);
                vTaskResume(send_h);
                vTaskDelay(10);
                vTaskResume(stuck_h);
                printf("motor");

                l++;
            }

            if (goal == 1)
            {
                lfs.file_close(&file);
                lfs.unmount();
              printf("mision done");
                vTaskEndScheduler();
            }

            lfs.file_write(&file, &log_w, sizeof(log_w));
           // printf("write log\n");
            i++;
        }
        vTaskDelayUntil(&llk_log, pdMS_TO_TICKS(2000));
        // vTaskDelay(2000);vTaskSuspend(NULL);
    }
}

double distance(double x1, double y1, double x2, double y2)
{
    /*
      pointA(lng x1, lat y1), pointB(lng x2, lat y2)
      D = Rcos^-1(siny1siny2 + cosy1cosy2cosΔx)
      Δx = x2 - x1
      R = 6378.137[km]
    */
    return EARTH_RAD * acos(sin(y1 * rp) * sin(y2 * rp) + cos(y1 * rp) * cos(y2 * rp) * cos(x2 * rp - x1 * rp));
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
    int r = 1;
    int32_t gga_s[4];

    double arctan;
    double torig[4];
    float yaw;

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

   // printf("task_gnss");

    vTaskResume(imu_h);
    vTaskResume(log_h);
    vTaskResume(landing_h);

    TickType_t llk_gnss;
    llk_gnss = xTaskGetTickCount();
    while (1)
    {
        call_a = call;
        ch = uart_getc(UART);
        nmeap_parse(&nmea, ch);

        if (call != call_a)
        {
            xEventGroupSetBits(gcs_event, gbit);
            vTaskDelayUntil(&llk_gnss, pdMS_TO_TICKS(100));
            if (xQueueReceive(imu_q, &yaw, 10) == 1)
            {
                arctan = atan2(goal_longitude - gga.longitude, goal_latitude - gga.latitude);
                torig[0] = cos(arctan);
                torig[1] = sin(arctan);
                torig[2] = cos(yaw);
                torig[3] = sin(yaw);
                gdata[0].arg = atan2(-torig[3] * torig[0] + torig[2] * torig[1], torig[2] * torig[0] + torig[3] * torig[1]);
            }
            else
            {
                gdata[0].arg = 0;
            }
            gdata[0].gns[0] = gga.latitude * 10000000;
            gdata[0].gns[1] = gga.longitude * 10000000;
            gdata[0].dis = distance(goal_longitude, goal_latitude, gga.longitude, gga.latitude);

         //   printf("%d,%d", gdata[0].dis, gdata[0].arg);

            xQueueSend(gnss_q, &gdata[0], 0);
            vTaskDelayUntil(&llk_gnss, pdMS_TO_TICKS(150));

        }
    }
}

void task_imu(void *imu_t)
{
    imu_q = xQueueCreate(512, sizeof(float));

    float euler[3];
    // for debug
   // printf("task_imu");
    vTaskSuspend(NULL);

    TickType_t llk_imu;
    llk_imu = xTaskGetTickCount();
    while (1)
    {

        if (xSemaphoreTake(xMutex, (TickType_t)0xffffff) == 1)
        {
            imu.update();
            imu.getAttEuler(euler);
            xQueueSend(imu_q, &euler[2], 10);
            // printf("imu");
            xSemaphoreGive(xMutex);
        }

        vTaskDelayUntil(&llk_imu, pdMS_TO_TICKS(20));
        // vTaskDelay(20);
    }
}

void task_gcontrol(void *gcontrol)
{
    float p;

    uint32_t ot[2];

    double dis_m;
    double arg_m;

    uint8_t l = 0;
    uint8_t s = 0;

    motor.init(pin_left_begin, pin_right_begin);

    motor.setDirForward(-1, 1);

//    printf("task_gcontrol");

    vTaskSuspend(NULL);

    TickType_t llk_gcontrol;
    llk_gcontrol = xTaskGetTickCount();

    while (1)
    {

        if (gnss_q != NULL && press_q != NULL && imu_q != NULL && receive_q != NULL)
        {
        //    printf("move\n");

            // calculate argument between ikafly and goal

            if (xQueueReceive(receive_q, &ot, 0) != 1)
            {
                ot[0] = 1000;
                ot[1] = 1;
            }

            if (xQueueReceive(gnss_q, &gdata[2], 200) == 1)
            {
            //    printf("gogo");

                dis_m = gdata[2].dis;
                arg_m = gdata[2].arg;

                if (dis_m > 0)
                {

                    p = arg_m * 39.15;

                    if (p >= 123)
                    {
                        p = 123;
                    }
                    else if (p <= -123)
                    {
                        p = -123;
                    }

                    if (dis_m < 20)
                    {
                        g_con = 1;
                        motor.stop();
                        g_con = 1;
                        vTaskSuspend(receive_h);
                        vTaskSuspend(send_h);
                        vTaskResume(ccontrol_h);
                        printf("gdone\n");
                        vTaskSuspend(NULL);
                    }

                    if (dis_m - ot[0] < 0)
                    {
                        motor.forward(900 + p, 900 - p);
                    }
                    else if (ot[1] == 1)
                    {
                        motor.forward(900 + p, 900 - p);
                    }
                    else
                    {
                        motor.stop();
                    }
                }
            }

            // for debug
            s++;
            if (s == 30)
            {

                g_con = 1;
                motor.stop();
                vTaskSuspend(receive_h);
                vTaskSuspend(send_h);
                vTaskResume(ccontrol_h);
                printf("gdone\n");
                vTaskSuspend(NULL);
            }
        }
        printf("gmotor");
        vTaskDelayUntil(&llk_gcontrol, pdMS_TO_TICKS(200));


        // for debug
    }
}

uint i = 1;
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

    vTaskDelay(1);
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
    i++;

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
    cam.init();
    cam.enableJpeg();

    // for debug
    // motor.init(pin_left_begin, pin_right_begin);
    // motor.setDirForward(-1, 1);

    uint32_t cnt = 0;
    uint8_t vert[5] = {0};

   // printf("task_ccontrol\n");
    vTaskSuspend(NULL);

    TickType_t llk_ccontrol;
    llk_ccontrol = xTaskGetTickCount();

    while (1)
    {
      //  printf("wate");
        result = xEventGroupWaitBits(gcs_event, cbit, pdTRUE, pdTRUE, (TickType_t)0xffffff);
        if (result && cbit)
        {
            if (xSemaphoreTake(xMutex, (TickType_t)0xffffff) == 1)
            {
               // printf("%d: ", cnt++);
                cam.capture();
                vTaskDelayUntil(&llk_ccontrol, pdMS_TO_TICKS(1000));

                uint32_t size = cam.getJpegSize();
               // printf("last: ");
                for (uint32_t i = size - 10; i < size; i++)
                //    printf("%02x", cam.image_buf[i]);
              //  printf(", size: %d\n", size);

                memset(red_area, 0, sizeof(red_area));
                JRESULT res = tjpg.prepare(cam.image_buf, size);

                if (res == JDR_OK)
                {
                 //   printf("Image size is %u x %u.\n%u bytes of work area is free.\n", tjpg.jdec.width, tjpg.jdec.height, tjpg.jdec.sz_pool);
                    res = tjpg.decomp(my_out_func, 3);
                    // 160x120 -> (1/2^3) -> 20x15
                    if (res == JDR_OK)
                    {
                     //   printf("\rDecompression succeeded.\n");
                        // print red_area (20x15)
                        /* uint8_t bit;
                         for (uint8_t y = 0; y < 15; y++)
                         {
                             //                   std::cout << std::bitset<32>(red_area[y]) << std::endl;
                             for (int8_t i = 20 - 1; i >= 0; i--)
                             {
                                 bit = (red_area[y] >> i) & 1;
                                 printf("%u", bit);
                             }
                             printf("\n");
                         }*/
                        count_vert(vert, red_area);

                        if (vert[0] > 30 && vert[1] > 30 && vert[2] > 30 && vert[3] > 30 && vert[4] > 30)
                        {
                            goal = 1;
                        //    printf("goal");
                            motor.stop();
                            xSemaphoreGive(xMutex);
                            vTaskSuspend(NULL);
                        }

                        int8_t dire = 0;
                        int8_t most = vert[0];

                        for (int8_t q = 0; q < 5; q++)
                        {
                            if (vert[q] > most)
                            {
                                most = vert[q];
                                dire = q;
                            }
                        }

                        switch (dire)
                        {

                        case 0:

                            if (vert[0] == 0)
                            {
                                motor.forward(300, 1023);
                            }
                            else
                            {
                                motor.forward(1023, 600);
                            }

                            break;

                        case 1:
                            if (vert[0])
                            {
                                motor.forward(1023.0);
                            }
                            else
                            {
                                motor.forward(1023, 800);
                            }

                            break;

                        case 2:

                            motor.forward(1023, 600);

                            break;

                        case 3:

                            motor.forward(1023, 1023);

                            break;

                        case 4:

                            motor.forward(600, 1023);

                            break;

                        default:

                            motor.forward(800, 1023);
                        }
                    }
                    else
                    {
                      //  printf("残念");
                    }
                }
                xSemaphoreGive(xMutex);
            }
           // printf("WWWW");
        }
         printf("cmotor");
        vTaskDelayUntil(&llk_ccontrol, pdMS_TO_TICKS(200));
    }
}

void task_send(void *send)
{

    uint8_t packet_s[32] = {0};
  //  printf("task_send\n");

    uint8_t st_r;
    uint32_t p = 0;

    vTaskSuspend(NULL);

    TickType_t llk_send;
    llk_send = xTaskGetTickCount();

    while (1)
    {
        // for debug

        if (gnss_q != NULL && press_q != NULL && imu_q != NULL)
        {

            if (xQueueReceive(gnss_q, &gdata[3], 10) == 1)
            {
                packet_s[0] = gdata[3].dis;
            }

            if (xQueueReceive(stuck_q, &st_r, 10) == 1)
            {
                packet_s[1] = st_r;
            }
            else
            {
                packet_s[1] = 0;
            }

            radio.send(packet_s);
           // printf("send");
        }
        vTaskDelayUntil(&llk_send, pdMS_TO_TICKS(2000));
        // vTaskDelay(2000);
    }
}

void task_receive(void *receive)
{

    uint32_t rec[2];
    uint8_t packet_r[32];
  //  printf("task_receive\n");

    vTaskSuspend(NULL);

    TickType_t llk_receive;
    llk_receive = xTaskGetTickCount();

    while (1)
    {
       // printf("rerere\n");
        radio.receive(packet_r);
      //  printf("ceive");

        rec[0] = packet_r[0];
        rec[1] = packet_r[1];

        xQueueSend(receive_q, &rec, 10);

        vTaskDelayUntil(&llk_receive, pdMS_TO_TICKS(3000));
        // vTaskDelay(3000);
    }
}
void task_stuck(void *stuck)
{

    uint8_t p = 0;
    uint8_t stucking = 0;

    stuck_q = xQueueCreate(32, sizeof(stucking));

  //  printf("task_stucking\n");

    // for debug
    vTaskSuspend(NULL);

    TickType_t llk_stuck;
    llk_stuck = xTaskGetTickCount();

    while (1)
    {
        if (gnss_q != NULL && press_q != NULL && imu_q != NULL)
        {

            if (xQueueReceive(gnss_q, &gdata[4], 10) == 1)
            {

                if (p == 0)
                {
                    gdata[5].dis = gdata[4].dis;
                    gdata[5].arg = gdata[4].arg;
                    p++;
                }
                else
                {
                    if (gdata[4].dis - gdata[5].dis < 10 || gdata[5].dis - gdata[4].dis < 10)
                    {
                        if (gdata[4].arg - gdata[5].arg < thrty || gdata[5].arg - gdata[4].arg < thrty)
                        {
                            if (g_con == 0)
                            {
                                vTaskSuspend(gcontrol_h);
                            }
                            else
                            {
                                vTaskSuspend(ccontrol_h);
                            }

                            stucking = 1;
                           // printf("stucking");
                            motor.backward(1023);
                            vTaskDelayUntil(&llk_stuck, pdMS_TO_TICKS(10000));

                            if (g_con == 0)
                            {
                                vTaskResume(gcontrol_h);
                            }
                            else
                            {
                                vTaskResume(ccontrol_h);
                            }
                        }
                    }
                    gdata[5].dis = gdata[4].dis;
                    gdata[5].arg = gdata[4].arg;
                }
                 xQueueSend(stuck_q, &stucking, 10);
            }
        }

       
       // printf("st\n");
        vTaskDelayUntil(&llk_stuck, pdMS_TO_TICKS(10000));
        //  vTaskDelay(3000);
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C, 400 * 1000);
    gpio_set_function(pin_i2c1_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_i2c1_scl, GPIO_FUNC_I2C);

    xMutex = xSemaphoreCreateMutex();
    gcs_event = xEventGroupCreate();
    receive_q = xQueueCreate(32, sizeof(uint32_t));

    imu.init();
    imu.setDebugPrint(false);
    // imu.calibration();

    radio.init();
   // printf("init");

    xTaskCreate(task_gnss, "task_gnss", 1024 * 3, NULL, 2, &gnss_h);
    xTaskCreate(task_imu, "task_imu", 1024 * 3, NULL, 4, &imu_h);
    xTaskCreate(task_landing, "task_landing", 1024 * 3, NULL, 5, &landing_h);

    xTaskCreate(task_gcontrol, "task_gcontrol", 1024 * 3, NULL, 6, &gcontrol_h);
     xTaskCreate(task_send, "task_send", 1024 * 3, NULL, 7, &send_h);
     xTaskCreate(task_receive, "task_receive", 1024 * 3, NULL, 8, &receive_h);
   

    
    xTaskCreate(task_ccontrol, "task_ccontrol", 1024 * 3, NULL, 7, &ccontrol_h);
    xTaskCreate(task_log, "task_log", 1024 * 3, NULL, 9, &log_h);
    xTaskCreate(task_stuck, "task_stuck", 1024 * 3 NULL, 10, &stuck_h);

    vTaskStartScheduler();
    while (1)
    {
    };
}
