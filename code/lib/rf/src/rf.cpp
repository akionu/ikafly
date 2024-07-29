#include "rf.h"
#include "hardware/timer.h"
#include "pico/platform.h"
#include "../../freertos/config/FreeRTOSConfig.h"
#include "../../freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../freertos/FreeRTOS-Kernel/include/task.h"



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

bool Radio::is_air_clear(){
	int8_t n = 0;
	for (int8_t i = 0; i < 32; i++) {
		if (receiveBit() == 0x00) n++;
	}
	return (n==32);
}

void Radio::send(uint8_t packet[32]) {

	// send start signal
	sendByte(0x01); // wakeup
	sendByte(0x01);
	sendByte(0x66);
	sendByte(0x99);

	for (uint8_t i = 0; i < 32; i++) {
		//		printf("[i]: %c: %d_10: 0x%02x: 0b", packet[i], packet[i], packet[i]);
		sendByte(packet[i]);
		//		printf("\n");
	}
	sendByte(0x00);
}

void Radio::sendByte(uint8_t data) {
	uint32_t before = 0;
	for (int8_t i = 7; i >= 0; i--) {
		before = time_us_32() + 1000;
//		printf("%c", (packet[i] & (1<<i))==0?'0':'1');
		gpio_put(pin_mosi, (data & (1<<i)));
		while (time_us_32() < before) tight_loop_contents();
	}
}



uint8_t Radio::receive(uint8_t packet[32]) {
	// デフォルトは1、ビットは反転してない
	uint8_t r[16] = {0x7f}; // 簡単のため富豪的
	uint8_t detect[2] = {0};
	while (1) {
		detect[0] = 0; detect[1] = 0;

		// fifo
		for (int8_t i = 1; i < 16; i++) {
			r[i-1] = r[i];
		}
		r[15] = receiveBit();
	
		for (int8_t i = 0; i < 8; i++) {
			detect[0] |= (r[i]<<(7-i));
			detect[1] |= (r[8+i]<<(7-i));
		}

		

		if (detect[0] == 0x66 && detect[1] == 0x99) break;
		printf("detect\n");
		vTaskDelay(5000);
	}

	for (int16_t i = 0; i < 32; i++) {
		packet[i] = receiveByte();
	}
	printf("detect signal:\n");
	return 1;
}

uint8_t Radio::receiveByte() {
	uint8_t c = 0;
	for (int8_t j = 7; j >= 0; j--) {
		//		printf("%c", (st==0)?'0':'1');
		c |= (receiveBit()<<j);
	}
	//	printf("0x%x", c);
	return c;
}

// return 0x00 or 0x01
uint8_t Radio::receiveBit() {
	uint8_t s = 0;
	uint8_t st = 0;
	uint32_t before = 0;
	for (int8_t k = 0; k < 5; k++) {
		before = time_us_32()+200;
		st = gpio_get(pin_miso);
		s += (st==0)?0:1; // ビット反転してない
		while (time_us_32() < before) tight_loop_contents();
	}
	s = (s>=3)?1:0; // 多数決符号
	return s;
}
