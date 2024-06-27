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

    uint8_t test_file = 0;
	bool ret = false;

	// prepare dummy data
	for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) {
        wbuf[i] = FLASH_PAGE_SIZE-i;
        wbuf[FLASH_PAGE_SIZE+i] = i;
    }
    for (uint16_t i = 0; i < TEKITOU_NUM; i++) wbuf[FLASH_PAGE_SIZE*2+i] = TEKITOU_NUM-i;

    // init should be called first (otherwise, hang up)
    ret = log1.init(); printf("init: ret: %s\n", ret?"ok":"fail");
    log1.nuke();
    ret = log1.mount(); printf("mount: ret: %s\n", ret?"ok":"fail");
    if (!ret) {
        // mount fails
	    // erase
        ret = log1.unmount(); printf("unmount: ret: %s\n", ret?"ok":"fail");
        ret = log1.nuke();
    	printf("%d: nuke...", time_us_32()/1000);
    	printf("%s\n", (ret?"ok":"fail"));
        ret = log1.make(&test_file, "somedat", 16);
        printf("make: ret: %s\n", ret?"ok":"fail");
        ret = log1.make(&test_file, "test", 0xff*3);
        printf("make: ret: %s\n", ret?"ok":"fail");
    }
    ret = log1.find(&test_file, "test", sizeof("test")-1); // sizeof("test") == 5 (because of last '\0')
//    printf("sizeof(\"test\"): %d", sizeof("test"));
    printf("find: ret: %s\n", ret?"ok":"fail");
    if (!ret) {
        // find fails
        printf("fails to find file\n");
        while (1) {}
    }

    printf("test_file: %d\n", test_file);

	printf("PICO_OK: %d, PICO_ERROR_NOT_PERMITTED: %d, PICO_ERROR_INSUFFICIENT_RESOURCES: %d\n", PICO_OK, PICO_ERROR_NOT_PERMITTED, PICO_ERROR_INSUFFICIENT_RESOURCES);
	// read data
	printf("%d: read...", time_us_32()/1000);
    ret = log1.read(test_file, rbuf, SIZE);
	printf("%s\n", "ok");
	printf("read data:\n 0x");
	for (uint16_t i = 0; i < SIZE; i++) printf("%02x", rbuf[i]);
	printf("\n");

	// write data
	printf("%d: write...", time_us_32()/1000);
    assert(SIZE==sizeof(wbuf));
	ret = log1.write(test_file, wbuf, SIZE);
//    ret = log1.append(test_file, &wbuf[SIZE-TEKITOU_NUM], TEKITOU_NUM);
	printf("%s\n", (ret?"ok":"fail"));

	// read data
	printf("%d: read...", time_us_32()/1000);
    ret = log1.read(test_file, rbuf, SIZE);
	printf("%s\n", "ok");

	printf("(potentially) written data:\n 0x");
	for (uint16_t i = 0; i < SIZE; i++) printf("%02x", wbuf[i]);
	printf("\n");
	printf("read data:\n 0x");
	for (uint16_t i = 0; i < SIZE; i++) printf("%02x", rbuf[i]);
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

    // save fs
    log1.save_fs_force();

    log1.unmount();
    log1.mount(); printf("mount: ret: %s\n", ret?"ok":"fail");

	while (1) vTaskDelay(100);
}

int main(void) {
	stdio_usb_init();
	sleep_ms(2000);
	printf("\n%s\n", __FILE_NAME__);

	xTaskCreate(task_flash, "task_flash", 1024, NULL, 1, NULL);
	vTaskStartScheduler();
	while(1) {}
}
