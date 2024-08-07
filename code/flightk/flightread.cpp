
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "ulog.h"

#define BUF_SIZE 5000

Log logging(0);
uint8_t imgbuf[BUF_SIZE] = {0};

int main(void) {
    stdio_usb_init();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);

    logging.init();
    logging.showAll();

    int16_t imgsize = 0;
    bool ret = logging.readImg(imgbuf, &imgsize, BUF_SIZE);
    if (ret) {
        printf("image found!:\n");
        for (int16_t i = 0; i < imgsize; i++) printf("%02x", imgbuf[i]);
    } else {
        printf("image not found\n");
    }
    printf("\n");

    while(1) {}
}
