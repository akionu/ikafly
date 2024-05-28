#include <stdio.h>
#include "pico/stdlib.h"

class Radio {
	public:
		Radio(uint8_t pin_mosi = 24, uint8_t pin_miso = 22);
		void init();
		bool is_air_clear();
		void send(uint8_t packet[32]);
		void receive(uint8_t packet[32]);
		uint8_t receiveChar();
		
	private:	
		uint8_t pin_mosi;
		uint8_t pin_miso;
};
