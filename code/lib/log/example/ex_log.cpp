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
#include "log.hpp"

Logging log1;

void task_flash(void* p);
void task_flash(void *p) {
    const uint16_t TEKITOU_NUM = 23;
    const uint16_t SIZE = FLASH_PAGE_SIZE*2+TEKITOU_NUM;
	// shoud NOT be static when running this tasks more than one
	static uint8_t wbuf[SIZE] = {0};
	static uint8_t rbuf[SIZE] = {0};

    uint8_t file0 = 0, file1 = 0;

	bool ret = false;

	// prepare dummy data
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) {
        wbuf[i] = FLASH_PAGE_SIZE-i;
        wbuf[FLASH_PAGE_SIZE+i] = i;
    }
    for (uint16_t i = 0; i < TEKITOU_NUM; i++) wbuf[FLASH_PAGE_SIZE*2+i] = TEKITOU_NUM-i;

    log1.init();
    log1.make(&file0, "somedata", 16);
    log1.make(&file1, "test", SIZE);

    printf("file0: %d, file1: %d\n", file0, file1);
    log1.open(1);

	printf("PICO_OK: %d, PICO_ERROR_NOT_PERMITTED: %d, PICO_ERROR_INSUFFICIENT_RESOURCES: %d\n", PICO_OK, PICO_ERROR_NOT_PERMITTED, PICO_ERROR_INSUFFICIENT_RESOURCES);
	// read data
	printf("%d: read page...", time_us_32()/1000);
    ret = log1.read(file1, rbuf, SIZE);
	printf("%s\n", "ok");
	printf("read data:\n 0x");
	for (uint16_t i = 0; i < SIZE; i++) printf("%x", rbuf[i]);
	printf("\n");

	// erase
	printf("%d: erase sector...", time_us_32()/1000);
	ret = log1.erase_sector_safe(log1.file_list[file0].offset); //tmp
	printf("%s\n", (ret?"ok":"fail"));

	// write data
	printf("%d: write page...", time_us_32()/1000);
	ret = log1.write(file1, wbuf, SIZE);
	printf("%s\n", (ret?"ok":"fail"));

	// read data
	printf("%d: read page...", time_us_32()/1000);
    ret = log1.read(file1, rbuf, SIZE);
	printf("%s\n", "ok");

	printf("(potentially) written data:\n 0x");
	for (uint16_t i = 0; i < SIZE; i++) printf("%x", wbuf[i]);
	printf("\n");
	printf("read data:\n 0x");
	for (uint16_t i = 0; i < SIZE; i++) printf("%x", rbuf[i]);
    printf("\n");

    uint16_t failcnt = 0;
    printf("checking...");
    for (uint16_t i = 0; i < SIZE; i++) {
        if (wbuf[i] != rbuf[i]) {
            printf("%d,", i);
            failcnt++;
        }
    }
    printf("%s(cnt:%d)", (failcnt==0)?"ok":"fail", failcnt);
    printf("\n");

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
