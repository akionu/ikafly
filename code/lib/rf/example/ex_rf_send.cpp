#include <stdio.h>
#include "pico/stdio.h"
#include "../src/rf.h"

Radio radio(24, 22);
uint8_t packet[32] = {0};

int main(void)
{
	stdio_init_all();
	sleep_ms(1000);
	printf("%s\n", __FILE_NAME__);

	radio.init();

	// dummy packet
	/*for (int8_t i = 0; i < 32; i++)
	{
		packet[i] = 'A' + i;
		packet[i] = 0x00;
	}
	for (int8_t i = 0; i < 32; i += 8)
	{
		for (int8_t j = 0; j < 8; j++)
		{
			packet[i + j] = (1 << (7 - j));
		}
	}*/
	// for (int i = 0; i <= 31; i++)
	//{
	//	packet[i] = 1;
	// }


uint32_t time;
	while (1)
	{
		printf("%dms: ", time_us_32() / 1000);
		
		time=time_us_32();

		 for(uint8_t i=0;i<4;i++){
                packet[i]=time>>8*(3-i);
            }
		for (int8_t i = 0; i < 32; i++)
			printf("%x", packet[i]);
		printf("\n");
		radio.send(packet);
		sleep_ms(1000);
	}

	return 0;
}
