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
#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "../../lib/log/src/littlefs.hpp"
#include "../../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../../lib/ikafly_pin.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <cmath>

uint32_t atai[1000]={0};

LFS lfs;
// variables used by the filesystem
lfs_file_t file;

void read_log(void *p){
    lfs.init();

 

    lfs.mount();
    lfs.file_open(&file, "latitude_f", LFS_O_RDONLY);
    lfs.file_read(&file, &atai,sizeof(atai));

     lfs.file_clode(&file);
    lfs.unmount();

    for(int i=0;i<100;i++){
        printf("%d",atai[i]);
    }

    while(1){};

}



int main(void){
    xTaskCreate(read_log,"read_log",256,NULL,1,NULL)
    vTaskStartScheduler();
    while(1){}
}