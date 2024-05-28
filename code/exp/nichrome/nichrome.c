#include <stdio.h>
#include "pico/stdlib.h"
#include "../../lib/ikafly_pin.h"

void nichrome_onoff(uint8_t pin, uint8_t seconds) {
	printf("ON for %ds\n", seconds);
	gpio_put(pin, 1);
	sleep_ms(seconds * 1000);
	gpio_put(pin, 0);
	printf("OFF\n");
	printf("wait a few seconds for the next heat...\n");
	sleep_ms(2000);
}

uint8_t ctoi(uint8_t c) {
	if (c >= '0' && c <= '9') return (c - '0');
	else return 0;
}

int main(void) {
	stdio_init_all();
	sleep_ms(2000);
	printf("Ikafly %s.c", __FILE_NAME__);

	gpio_init(pin_nichrome_left);
	gpio_set_dir(pin_nichrome_left, GPIO_OUT);

	printf("nichrome ready.\npress \"1\" to heat.\n");
	uint8_t c = 0;
	while (1) {
		c = getchar_timeout_us(10 * 1000);
//		printf("%x\n", c);
		if (c == 0 || c == 0xff) continue;
		else nichrome_onoff(pin_nichrome_left, ctoi(c));
		c = 0;
		sleep_ms(10);
	}
}
