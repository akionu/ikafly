#include "rf.h"
#include <cstdint>

Radio::Radio(uint8_t pin_mosi, uint8_t pin_miso) {
	this->pin_mosi = pin_mosi;
	this->pin_miso = pin_miso;
}

void Radio::init() {
	gpio_init(pin_mosi);
	gpio_init(pin_miso);
	gpio_set_dir(pin_mosi, GPIO_OUT);
	gpio_set_dir(pin_miso, GPIO_IN);
}

bool Radio::is_air_clear() {
	return ((gpio_get(pin_miso) == 0)?true:false);
}

void Radio::send(uint8_t packet[32]) {
	for (int8_t i = 0; i < 32; i++) {
		printf("%c", packet[i]);
	}
	printf("\n");
	// send start signal 11110
	for (int8_t i = 0; i < 4; i++) {
		gpio_put(pin_mosi, 1);
		sleep_ms(1);
	}
	gpio_put(pin_mosi, 0);
	sleep_ms(1);

	for (int16_t i = 0; i < 32; i++) {
//		printf("[i]: %c: 0x%02x: 0b", packet[i], packet[i]);
		for (int8_t j = 0; j < 8; j++) {
//			printf("%c", (packet[i] & (1<<j))==0?'0':'1');
			gpio_put(pin_mosi, (packet[i] & (1<<j)));
			sleep_ms(1);
		}
//		printf("\n");
	}
}

void Radio::receive(uint8_t packet[32]) {
	int8_t n = 0;
	// 11110 => start signal
	while (1) {
		if (gpio_get(pin_miso) != 0) n++;
		else if (n == 4) break;
		sleep_ms(1);
	}
	printf("detect signal:\n");

	for (int16_t i = 0; i < 32; i++) {
		packet[i] = receiveChar();
	}
}

uint8_t Radio::receiveChar() {
	uint8_t c = 0;
	for (int8_t j = 0; j < 8; j++) {
		uint8_t st = gpio_get(pin_miso);
		printf("%c", (st==0)?'0':'1');
		c |= (1<<j);
		sleep_ms(1);
	}
	return c;
}
