
#include <cstdio>
#include <cstring>
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

#define SIZEOF_MEM(s) (sizeof(s)-1)

Logging logf;

void task_log(void* p) {
    bool ret = false, found = false;
    Logging::file_num_t file_boot_count = 0xff;
    uint32_t boot_count = 1000;
//    uint8_t boot_count[] = "hello world";
//    uint64_t boot_count = 1000;

    ret = logf.init();
    printf("init: %s\n", ret?"ok":"fail");
//    logf.nuke();
    ret = logf.mount();
    printf("mount: %s\n", ret?"ok":"fail");
    found = logf.find(&file_boot_count, "bootcnt", SIZEOF_MEM("bootcnt"));
    printf("find: %s\n", found?"ok":"fail");
    if (!ret || !found || logf.fsdata_max_size[file_boot_count]==8) {
        ret = logf.unmount();
        printf("unmount: %s\n", ret?"ok":"fails");
        ret = logf.nuke();
        printf("nuke: %s\n", ret?"ok":"fails");
        ret = logf.make(&file_boot_count, "bootcnt", sizeof(boot_count));
        printf("make: %s\n", ret?"ok":"fails");
        ret = logf.write(file_boot_count, (void*)&boot_count, sizeof(boot_count));
    }
    ret = logf.read(file_boot_count, (void*)&boot_count, sizeof(boot_count));
    printf("read: %s: %d\n", ret?"ok":"fails", boot_count);

    boot_count++;
//    memcpy(&boot_count, "oolong tea", sizeof(boot_count)-1);
//    boot_count = 0x123345675643;
    printf("boot_count: %d\n", boot_count);

    ret = logf.write(file_boot_count, (void*)&boot_count, sizeof(boot_count));
    printf("write: %s\n", ret?"ok":"fails");

    ret = logf.read(file_boot_count, (void*)&boot_count, sizeof(boot_count));
    printf("read: %s: %d\n", ret?"ok":"fails", boot_count);
    logf.save_fs_force();
    printf("save fs: %s\n", ret?"ok":"fails");

    while(1) {}
}

int main(void) {
	stdio_usb_init();
	sleep_ms(1000);
	printf("\n%s\n", __FILE_NAME__);

	xTaskCreate(task_log, "task_log", 1024, NULL, 1, NULL);
	vTaskStartScheduler();
	while(1) {}
}
