#include <stdio.h>
#include "hardware/timer.h"
#include "pico/stdio.h"
#include "../src/rf.h"

Radio radio(24, 22);
uint8_t packet[32] = {0};

int main(void) {
	stdio_init_all();
	sleep_ms(2000);
	printf("%s\n", __FILE_NAME__);

	radio.init();

	while (1) {
//		uint8_t st = radio.receiveBit();
//		printf("%c", (st==0)?'0':'1');
		radio.receive(packet);
		printf("%dms: ", time_us_32()/1000);
		for (int8_t i = 0; i < 32; i++) {
			printf("%x", packet[i]);
		}
		printf("\n");

	}


	return 0;
}
