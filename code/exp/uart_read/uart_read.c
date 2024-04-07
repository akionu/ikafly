#include <stdint.h>
#include <stdio.h>
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13

static void on_uart_rx() {
    while (uart_is_readable(UART)) {
        uint8_t ch = uart_getc(UART);
		printf("%c", ch);
    }
}

int main(void) {
	stdio_init_all();

    // uart setting
    uart_init(UART, 9600);
    gpio_set_function(MOSI, GPIO_FUNC_UART);
    gpio_set_function(MISO, GPIO_FUNC_UART);
    uart_set_baudrate(UART, 9600);
    uart_set_hw_flow(UART, false, false);
    uart_set_format(UART, 8, 1, UART_PARITY_NONE);
    //uart_set_fifo_enabled(UART, true);
    uart_set_fifo_enabled(UART, false);
    irq_set_exclusive_handler(IRQ, on_uart_rx);
    irq_set_enabled(IRQ, true);
    uart_set_irq_enables(UART, true, false);

	while(1) tight_loop_contents();
}
