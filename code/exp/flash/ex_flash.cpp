#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <stdio.h>

void erase_sector(const uint32_t offset) {
	// erase
	uint32_t ints = save_and_disable_interrupts();
	flash_range_erase(
		offset,
		FLASH_SECTOR_SIZE); // FLASH_SECTOR_SIZE == 4096Byte defined in flash.h
	restore_interrupts(ints);
}

void write_page(const uint32_t offset, uint8_t buf[FLASH_PAGE_SIZE]) {
	uint32_t ints = save_and_disable_interrupts();
	flash_range_program(offset, buf, FLASH_PAGE_SIZE);
	restore_interrupts(ints);
}

void read_page(const uint32_t offset, uint8_t buf[FLASH_PAGE_SIZE]) {
	uint32_t ints = save_and_disable_interrupts();
	const uint8_t *p = (const uint8_t *)(XIP_BASE + offset);

	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) {
		buf[i] = p[i];
	}
	restore_interrupts(ints);
}

int main(void) {
	const uint32_t OFFSET = 0x200000; // Block 32(just after first 2MiB)

	uint8_t wbuf[FLASH_PAGE_SIZE] = {0};
	uint8_t rbuf[FLASH_PAGE_SIZE] = {0};

	stdio_usb_init();
	sleep_ms(2000);
	printf("\n%s\n", __FILE_NAME__);

	// prepare dummy data
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++)
		wbuf[i] = (uint8_t)i;

	printf("offset: %x\n", OFFSET);
	printf("%d: erase flash...\n", time_us_32() / 1000);
	erase_sector(OFFSET);
	printf("%d: write dummy data...\n", time_us_32() / 1000);
	write_page(OFFSET, wbuf);
	printf("%d: read stored data...\n", time_us_32() / 1000);
	read_page(OFFSET, rbuf);

	printf("written data:\n 0x");
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++)
		printf("%x", wbuf[i]);
	printf("\n");
	printf("read data:\n 0x");
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++)
		printf("%x", rbuf[i]);

	while (1)
		tight_loop_contents();
}
