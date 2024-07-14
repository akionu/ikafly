#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "cam.h"
#include "tjpg.hpp"

#define SDA 14
#define SCL 15
#define I2C i2c1
#define XCLK 0
#define Y2_PIO_BASE 1
#define PIO pio0
#define PIO_SM 0
#define DMA_CH 0
Cam cam(I2C, XCLK, Y2_PIO_BASE, PIO, PIO_SM, DMA_CH);

TJPGD tjpg;

static int my_out_func(JDEC* jdec, void* bitmap, JRECT* rect) {
    const uint8_t N_BPP = 3; // RGB888
    uint8_t *src;
    uint16_t x, y;//, bws;
    uint8_t r,g,b;

//    printf("\n");
    src = (uint8_t*)bitmap;
//    bws = N_BPP * (rect->right - rect->left + 1);
    for (y = 0; y < (rect->bottom - rect->top +1); y++) {
        for (x = 0; x < (rect->right - rect->left + 1); x+=N_BPP) {
//            printf("y: %d\n", y);
            r = *(src + 3 * (y*(rect->right - rect->left + 1) + x));
            g = *(src + 3 * (y*(rect->right - rect->left + 1) + x) + 1);
            b = *(src + 3 * (y*(rect->right - rect->left + 1) + x) + 2);
//            printf("(%03d,%03d,%03d),", r,g,b);
        }
    }

    return 1;// Continue to decompress
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);

    i2c_init(I2C, 400*1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);

    cam.init();
    cam.enableJpeg();

    uint32_t cnt = 0;
    while (1) {
        printf("%d: ", cnt++);
        cam.capture();
        while (!cam.isCaptureFinished()) tight_loop_contents();
        uint32_t size = cam.getJpegSize();
        printf("last: ");
        for (uint32_t i = size-10; i < size; i++) printf("%02x", cam.image_buf[i]);
        printf(", size: %d\n", size);

        JRESULT res = tjpg.prepare(cam.image_buf, size);
        if (res == JDR_OK) {
            printf("Image size is %u x %u.\n%u bytes of work area is free.\n", tjpg.jdec.width, tjpg.jdec.height, tjpg.jdec.sz_pool);
            res = tjpg.decomp(my_out_func, 2);
            // 160x120 -> (1/2^2) -> 40x30
            if (res == JDR_OK) {
                printf("\rDecompression succeeded.\n");
            } else {
                printf("jd_decomp() failed (rc=%d)\n", res);
            }
        } else {
            printf("jd_prepare() failed (rc=%d)\n", res);
        }

        sleep_ms(1500);

    }

}

