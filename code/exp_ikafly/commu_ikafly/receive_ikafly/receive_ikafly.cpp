#include <stdio.h>
#include <math.h>
#include "hardware/timer.h"
#include "pico/stdio.h"
#include "../../../lib/rf/src/rf.h"
#include <math.h>

Radio radio(24, 22);
uint8_t packet[32] = {0};
uint8_t i;

uint32_t latitude_ot;
uint32_t longitude_ot;

void assemble_lat(uint8_t packet_r[32])
{
	 latitude_ot=0;
	 longitude_ot=0;


	for(i=0;i<4;i++){
		latitude_ot=latitude_ot|packet[i];
		if(i != 3){
			latitude_ot=latitude_ot<<8;
			}
	}
	for(i=0;i<4;i++){
		longitude_ot=longitude_ot|packet[i+4];
		if(i != 3){
			longitude_ot=longitude_ot<<8;
			}
	}
	
}

int main(void)
{
	stdio_init_all();
	sleep_ms(2000);
	printf("%s\n", __FILE_NAME__);

	radio.init();

	while (1)
	{
		//		uint8_t st = radio.receiveBit();
		//		printf("%c", (st==0)?'0':'1');
		radio.receive(packet);
		printf("%dms: ", time_us_32() / 1000);

		assemble_lat(packet);
		printf("lat=%d\n", latitude_ot);
		printf("lon=%d\n", longitude_ot);
		printf("\n");

		sleep_ms(1000);
	}
}
