#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

extern "C"
{
#include "../../../lib/gnss/nmeap-0.3/inc/nmeap.h"
}

#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13

 nmeap_context_t nmea;
 nmeap_gga_t gga;


/** called when a gpgga message is received and parsed */

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
}

int main(void)
{
    stdio_init_all();
    int ch;
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

    // uart setting
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
        ch = uart_getc(UART);
        nmeap_parse(&nmea, ch);
        printf("%f,%f",gga.latitude,gga.longitude);
        printf("受信中");
    }
}