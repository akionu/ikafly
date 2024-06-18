#include <stdio.h>
#include "hardware/timer.h"
#include "pico/stdio.h"
#include "../../../lib/rf/src/rf.h"
#include <math.h>

Radio radio(24, 22);
uint8_t packet[32] = {0};
float latitude_ot;

void assemble_lat(uint8_t atai[32]){
  latitude_ot=0;
  int atai_digit=8;
  int i;
  for(i=0;i<=7;i++){
	latitude_ot=latitude_ot+atai[i]*pow((double)10,(double)atai_digit-i-1);
  }
}

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
			//assemble_lat(packet);
		}
		    printf("\n");
	    }

}