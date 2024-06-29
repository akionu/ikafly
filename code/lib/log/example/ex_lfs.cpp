#include <cstdio>
#include <cstring>
#include <cstdlib>
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
#include "littlefs.hpp"

LFS lfs;
// variables used by the filesystem
lfs_file_t file;

void task_lfs(void* p) {
    int err = lfs.init();
    // mount the filesystem
    err = lfs.mount();

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs.format();
        lfs.mount();
    }

    // read current count
    uint32_t boot_count = 0;
    lfs.file_open(&file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs.file_read(&file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    lfs.file_rewind(&file);
    lfs.file_write(&file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs.file_close(&file);

    // release any resources we were using
    lfs.unmount();

    // print the boot count
    printf("boot_count: %d\n", boot_count);

    while(1) {}
}

int main(void) {
    stdio_usb_init();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);

    xTaskCreate(task_lfs, "task_lfs", 512, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1) {}
}

