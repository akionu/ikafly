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
#define EARTH_RAD 6378137
#define goal_long 
#define goal_lat 
#define h 0.01745329251 
double dis = 0;
uint32_t m = 0, n = 0;

nmeap_context_t nmea;
nmeap_gga_t gga;

int j[2] = {0};

/*double distance()
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
}*/

double cal_distance(double x1, double y1, double x2, double y2)
{
    /*
      pointA(lng x1, lat y1), pointB(lng x2, lat y2)
      D = Rcos^-1(siny1siny2 + cosy1cosy2cosΔx)
      Δx = x2 - x1
      R = 6378.137[km]
    */
    return EARTH_RAD * acos(sin(y1 * h) * sin(y2 * h) + cos(y1 * h) * cos(y2 * h) * cos(x2 * h - x1 * h));
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
                /* j[1] = gga.longitude * 10000000;
                 j[0] = gga.latitude * 10000000;
                 printf("%d\n%d\n", j[0], j[1]);
                 */

                dis = cal_distance(goal_long, goal_lat, gga.longitude, gga.latitude);
                printf("%f", dis);
            }
        }
    }
}
