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

#define I2C i2c1
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define RAD2DEG 57.2958
#define goal_long
#define goal_lat


 nmeap_context_t nmea;
 nmeap_gga_t gga;
 Radio radio(24, 22);
 uint8_t packet[20]={0};

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
}

 int main()
{
    int ch;
    stdio_init_all();
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);


    uart_init(UART, 9600);
    radio.init();
    gpio_set_function(MOSI, GPIO_FUNC_UART);
    gpio_set_function(MISO, GPIO_FUNC_UART);
    uart_set_baudrate(UART, 9600);
    uart_set_hw_flow(UART, false, false);
    uart_set_format(UART, 8, 1, UART_PARITY_NONE);


    uart_set_fifo_enabled(UART, true);
    irq_set_enabled(IRQ, true);
    uart_set_irq_enables(UART, true, false);

    while (1)
    {

        if(uart_is_readable(UART)){
            int i=0;
        while(uart_is_readable(UART)){
        ch = uart_getc(UART);
        packet[1]=ch;
        i++;
        }
        radio.send(packet);
        }
        
    }
}