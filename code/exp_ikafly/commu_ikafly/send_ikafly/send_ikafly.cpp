#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "../../../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../../../lib/ikafly_pin.h"
#include "../../../lib/rf/src/rf.h"
#include "FreeRTOSConfig.h"
#include "../../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../../lib/freertos/FreeRTOS-Kernel/include/task.h"

#define I2C i2c1
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define RAD2DEG 57.2958

int latitude_digit = 8;
int longitude_digit = 9;
float latitude_ot = 0;
float longitude_ot = 0;
int cgg = 0;
int cgg_o;

Radio radio(24, 22);
nmeap_context_t nmea;
nmeap_gga_t gga;
uint8_t packet[32] = {0};

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
}

void gnss(void *gn)
{
    int ch;
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

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
    printf("success init");

    TickType_t gnss_h;
    gnss_h = xTaskGetTickCount();

    while (1)
    {
        cgg_o = cgg;

        ch = uart_getc(UART);
        nmeap_parse(&nmea, ch);

        if (cgg_o != cgg)
        {
            vTaskDelayUntil(&gnss_h, pdMS_TO_TICKS(100));
        }
    }
}

void disassemble_lat(void *dis)
{
    radio.init();
    int latitude_digit = 9;
    int longitude_digit = 10;
    int i;
    
    uint32_t latitude_s;
    uint32_t longitude_s;
     
    TickType_t diss_h;
    diss_h = xTaskGetTickCount();

    while (1)
    {
        if (gga.latitude != 0.0)
        {

            latitude_s = gga.latitude * 10000000;
            longitude_s = gga.longitude * 10000000;

            for(i=0;i<4;i++){
                packet[i]=latitude_s>>8*(3-i);
            }
            for(i=0;i<4;i++){
                packet[i+4]=longitude_s>>8*(3-i);
            }

            printf("sending");
            radio.send(packet);
        }
        vTaskDelayUntil(&diss_h, pdMS_TO_TICKS(5000));
    }
}

int main()
{

    stdio_init_all();
    sleep_ms(1000);
    uart_init(UART, 9600);
    xTaskCreate(gnss, "gnss", 256, NULL, 1, NULL);
    xTaskCreate(disassemble_lat, "dis", 256, NULL, 2, NULL);
    vTaskStartScheduler();
    while (1)
        ;
}