#include <stdio.h>
#include <string.h>
#include "hardware/timer.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"

SemaphoreHandle_t flashMutex;

// ew stands for erase, write
struct flash_ew_t {
	uint32_t offset;
	const uint8_t* buf;
};

bool flashMutex_init() {
	flashMutex = xSemaphoreCreateMutex();
	return (flashMutex != NULL);
}

bool flashMutex_lock() {
	return (xSemaphoreTake(flashMutex, (TickType_t)20) == pdTRUE);
}

bool flashMutex_unlock() {
	return (xSemaphoreGive(flashMutex) == pdTRUE);
}

void erase_sector_unsafe(void* p) {
	uint32_t* offset = (uint32_t*)p;
	flash_range_erase(*offset, FLASH_SECTOR_SIZE); // FLASH_SECTOR_SIZE == 4096Byte defined in flash.h
}

void write_page_unsafe(void *p) {
	struct flash_ew_t* fp = (struct flash_ew_t*)p;
	flash_range_program(fp->offset, fp->buf, FLASH_PAGE_SIZE);
}

void read_page(const uint32_t offset, uint8_t* buf, uint32_t size) {
	const uint8_t *p = (const uint8_t *)(XIP_BASE + offset);
	memcpy(buf, p, size);
}

bool erase_sector_safe(const uint32_t offset) {
	if (flashMutex_lock()) {
		int8_t ret = flash_safe_execute(erase_sector_unsafe, (void*)&offset, UINT32_MAX);
		printf("locked, ret: %d ", ret);
		return (flashMutex_unlock() && (PICO_OK == ret));
	}
	return false;
}

bool write_page_safe(const uint32_t offset, uint8_t* buf) {
	struct flash_ew_t f = {
		.offset = offset,
		.buf    = buf
	};
	if (flashMutex_lock()) {
		int8_t ret = flash_safe_execute(write_page_unsafe, (void*)&f, UINT32_MAX);
		printf("locked, ret: %d ", ret);
		return (flashMutex_unlock() && (PICO_OK == ret));
	}
	return false;
}

void task_flash(void *p) {
	const uint32_t OFFSET = 0x200000; // Block 32(just after first 2MiB)

	// shoud NOT be static when running this tasks more than one
	static uint8_t wbuf[FLASH_PAGE_SIZE] = {0};
	static uint8_t rbuf[FLASH_PAGE_SIZE] = {0};

	bool ret = false;

	// prepare dummy data
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) wbuf[i] = FLASH_PAGE_SIZE-i;

	// init semaphore
	flashMutex_init();

	printf("Offset: 0x%x\n", OFFSET);
	printf("PICO_OK: %d, PICO_ERROR_NOT_PERMITTED: %d, PICO_ERROR_INSUFFICIENT_RESOURCES: %d\n", PICO_OK, PICO_ERROR_NOT_PERMITTED, PICO_ERROR_INSUFFICIENT_RESOURCES);
	// read data
	printf("%d: read page...", time_us_32()/1000);
	read_page(OFFSET, rbuf, FLASH_PAGE_SIZE);
	printf("%s\n", "ok");
	printf("read data:\n 0x");
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) printf("%x", rbuf[i]);
	printf("\n");

	// erase
	printf("%d: erase sector...", time_us_32()/1000);
	ret = erase_sector_safe(OFFSET);
	printf("%s\n", (ret?"ok":"fail"));

	// write data
	printf("%d: write page...", time_us_32()/1000);
	ret = write_page_safe(OFFSET, wbuf);
	printf("%s\n", (ret?"ok":"fail"));

	// read data
	printf("%d: read page...", time_us_32()/1000);
	read_page(OFFSET, rbuf, FLASH_PAGE_SIZE);
	printf("%s\n", "ok");

	printf("(potentially) written data:\n 0x");
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) printf("%x", wbuf[i]);
	printf("\n");
	printf("read data:\n 0x");
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) printf("%x", rbuf[i]);

	while (1) vTaskDelay(100);
}

int main(void) {
	stdio_usb_init();
	sleep_ms(2000);
	printf("\n%s\n", __FILE_NAME__);

	xTaskCreate(task_flash, "task_flash", 512, NULL, 1, NULL);
	vTaskStartScheduler();
	while(1) {}
}
