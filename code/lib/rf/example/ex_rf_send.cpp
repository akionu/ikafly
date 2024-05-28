#include <stdio.h>
#include "pico/stdio.h"
#include "../src/rf.h"

Radio radio(24, 22);
uint8_t packet[32] = {0};

int main(void) {
	stdio_init_all();
	sleep_ms(1000);
	printf("%s\n", __FILE_NAME__);

	radio.init();

	// dummy packet
	for (int8_t i = 0; i < 32; i++) {
		packet[i] = 'A'+i;
	}

	while (1) {
		printf("%d ms: ", time_us_32()/1000);
		radio.send(packet);
		sleep_ms(1000);
	}

	return 0;
}
