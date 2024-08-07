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
#include "../../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../../lib/ikafly_pin.h"

#define I2C i2c1
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define RAD2DEG 57.2958
#define goal_long 36.10377480555555
#define goal_lat 140.08785505555556
#define h 0.01745329251
double dis = 0;
uint32_t m = 0, n = 0;

nmeap_context_t nmea;
nmeap_gga_t gga;

uint32_t j[2] = {0};

double distance()
{
    double RX = 6378.137;
    double RY = 6356.752;

    double dx = goal_long * h - gga.longitude * h, dy = goal_lat * h - gga.latitude * h;
    double mu = (goal_lat * h + gga.latitude * h) / 2.0;
    double E = sqrt(1 - pow(RY / RX, 2.0));
    double W = sqrt(1 - pow(E * sin(mu), 2.0));
    double M = RX * (1 - pow(E, 2.0)) / pow(W, 3.0);
    double N = RX / W;
    return sqrt(pow(M * dy, 2.0) + pow(N * dx * cos(mu), 2.0));
}

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
    m++;
}

int main()
{
    int ch;

    stdio_init_all();
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
        if (uart_is_readable(UART))
        {
            m = n;
            ch = uart_getc(UART);
            nmeap_parse(&nmea, ch);

            if (m != n)
            {  
                j[1] = gga.longitude * 100;
                j[0] = gga.altitude;
                printf("%f\n%d\n", gga.altitude, j[1]);
            }
        }
    }
}
