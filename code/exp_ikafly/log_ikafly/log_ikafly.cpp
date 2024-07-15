#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "hardware/timer.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "FreeRTOSConfig.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "../../lib/log/src/littlefs.hpp"
#include "../../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../../lib/ikafly_pin.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <cmath>

LFS lfs;
// variables used by the filesystem
lfs_file_t file;

#define I2C i2c1
#define RAD2DEG 57.2958
#define pai 3.1415
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;
double arg;
int cgg = 0;
int cgg_o = 0;
int u = 0;
int i = 0;

uint32_t latitude_f = 0;

TaskHandle_t task;
TaskHandle_t gnss;
SemaphoreHandle_t xMutex;

nmeap_context_t nmea;
nmeap_gga_t gga;

void task_lfs(void *p)
{
    int st = 0;
    st = lfs.init();
    printf("init: %d\n", st);
    if (u == 0)
    {
        lfs.format();
        u++;
        printf("%d",u);
    }

    st = lfs.mount();
    printf("mount: %d\n", st);
    if (st < 0) {
        lfs.format();
        st = lfs.mount();
        printf("mount: %d\n", st);
    }

    st = lfs.file_open(&file, "latitude_f", LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
                printf("open: %d\n", st);

    TickType_t lastlog;
    lastlog = xTaskGetTickCount();

    while (1)
    {
        if(latitude_f >10000){
            if(xSemaphoreTake(xMutex, (TickType_t)0xfffff) == 1)
            {

                

                /* if (y)
                 {
                     printf("format");
                     lfs.format();
                     lfs.mount();
                     u++;
                 }*/

                


                //lfs.file_rewind(&file);

                printf("oo");
                st = lfs.file_write(&file, &latitude_f, sizeof(latitude_f));
                printf("write: %d\n", st);
                printf("kkk");

                st = lfs.file_sync(&file);
                printf("close: %d\n", st);

                printf("ppp");

                

                xSemaphoreGive(xMutex);
            
            }
        }
        
        vTaskDelayUntil(&lastlog, pdMS_TO_TICKS(1000));
    }
    //lfs.unmount();
}

uint32_t k = 0;

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
    cgg++;
}

void task_gnss(void *k)
{
    int ch;
    TickType_t last;
    last = xTaskGetTickCount();

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

    while (1)
    {
        if (xSemaphoreTake(xMutex, (TickType_t)0xfffff) == 1)
        {
            cgg_o = cgg;
            ch = uart_getc(UART);
            nmeap_parse(&nmea, ch);
            if (cgg_o != cgg && gga.latitude > 9)
            {
                latitude_f =(int)(gga.latitude * 10000000);
                xSemaphoreGive(xMutex);
                vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
            }else{
                xSemaphoreGive(xMutex);
            }
        }
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n%s\n", __FILE_NAME__);

     xMutex = xSemaphoreCreateMutex();

     xTaskCreate(task_gnss, "task_gnss", 512, NULL, 1, NULL);
     xTaskCreate(task_lfs, "task_lfs", 512, NULL, 2, &task);
   

    vTaskStartScheduler();
    while (1)
    {
    };
}
