#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "hardware/timer.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "FreeRTOSConfig.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "../lib/log/src/littlefs.hpp"
#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../lib/ikafly_pin.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <cmath>

uint32_t atai[1000] = {0};

LFS lfs;
// variables used by the filesystem
lfs_file_t file;

void read_log(void *p)
{
    printf("read");
    lfs.init();

    lfs.mount();
    lfs.file_open(&file, "ikafly", LFS_O_RDONLY);
    lfs.file_read(&file, &atai, 1000);

    lfs.file_close(&file);
    lfs.unmount();

    printf("can read");

    for (int i = 0; i < 1000; i++)
    {
            printf("i: %d: %d\n", i, atai[i]);
    }

    

    while (1)
    {
    };
}

int main(void)
{

    stdio_init_all();
    sleep_ms(2000);

    xTaskCreate(read_log, "read_log", 256, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1)
    {
    }
}