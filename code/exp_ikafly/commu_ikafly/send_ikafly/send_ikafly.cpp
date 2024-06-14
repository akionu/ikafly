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
Radio radio(24, 22);

nmeap_context_t nmea;
nmeap_gga_t gga;
uint8_t latitude_c[32]={0};

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
}


void disassemble_lat(float atai){
    int atai_digit=8;
    int i;
    
    atai=atai*1000000;
    for(i=0;i<8;i++){  
        latitude_c[i]=atai/pow((double)10,(double)atai_digit-i-1);
        atai=(int)atai%(int)pow((double)10,(double)atai_digit-i-1);
        printf("%d",latitude_c[i]);

    }
    
}


 int main(void)
{
    int ch;
    stdio_init_all();
    sleep_ms(1000);
    radio.init();
    sleep_ms(1000);
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);


    uart_init(UART, 9600);
    sleep_ms(1000);
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

    while (1)
    {  

        if(uart_is_readable(UART)){
        ch = uart_getc(UART);
        nmeap_parse(&nmea, ch);
        if(gga.latitude !=0){
        disassemble_lat(gga.latitude);
        
         radio.send(latitude_c);
        }

        }
    }
}
