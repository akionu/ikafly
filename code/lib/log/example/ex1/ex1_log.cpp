
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

void task_log(void* p) {
    log1.init();
    if (!log1.mount()) {
        log1.unmount();
        log1.nuke();
    }
    uint8_t f;
//    log1.make(&f, "test", 16);
//    log1.write(f, "flog jumps over the lazy dog", 16);
    log1.save_fs_force();
    log1.unmount();
    log1.mount();
    
    while(1) {}
}

int main(void) {
	stdio_usb_init();
	sleep_ms(2000);
	printf("\n%s\n", __FILE_NAME__);

	xTaskCreate(task_log, "task_log", 1024, NULL, 1, NULL);
	vTaskStartScheduler();
	while(1) {}
}
