#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#define PIN_GPS_ON 25 // INPUT LOW to ON
#define PIN_LED 29

int main(void) {
	stdio_init_all();
	sleep_ms(1000);
	printf("This is wakeup_seq.\n");
	
	gpio_init(PIN_GPS_ON);
	gpio_set_dir(PIN_GPS_ON, GPIO_OUT);
	gpio_put(PIN_GPS_ON, 1);

	gpio_init(PIN_LED);
	gpio_set_dir(PIN_LED, GPIO_OUT);
	gpio_put(PIN_LED, 0);

	sleep_ms(2000);
	gpio_put(PIN_GPS_ON, 0);

	while(1) {
		gpio_put(PIN_LED, 1);
		sleep_ms(250);
		gpio_put(PIN_LED, 0);
		sleep_ms(250);
	}

}
