#include <stdio.h>
#include "pico/stdlib.h"

class Radio {
	public:
		Radio(uint8_t pin_mosi = 24, uint8_t pin_miso = 22);
		void init();
		bool is_air_clear();
		void send(uint8_t packet[32]);
		void sendByte(uint8_t data);
		uint8_t receive(uint8_t packet[32]);
		uint8_t receiveByte();
		uint8_t receiveBit();
		
	private:	
		uint8_t pin_mosi;
		uint8_t pin_miso;
};
