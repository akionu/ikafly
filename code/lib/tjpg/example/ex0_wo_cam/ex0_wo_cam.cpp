#include <stdio.h>
#include "pico/stdlib.h"
#include "tjpg.hpp"
#include "img.h"

TJPGD tjpg;

static int my_out_func(JDEC* jdec, void* bitmap, JRECT* rect) {
    const uint8_t N_BPP = 3; // RGB888
    uint8_t *src;
    uint16_t y, bws;
    /* Progress indicator */
//    if (rect->left == 0) {
//        printf("\r%lu%%", (rect->top << jd->scale) * 100UL / jd->height);
 //   }

    src = (uint8_t*)bitmap;
    bws = N_BPP * (rect->right - rect->left + 1);
    for (y = rect->top; y <= rect->bottom; y++) {
        for (uint16_t i = 0; i < bws; i++) printf("%02x", src[i]);
        printf("\n");
        src += bws;
    }

    return 1;// Continue to decompress
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);

    JRESULT res = tjpg.prepare(img, sizeof(img));
    if (res == JDR_OK) {

        printf("Image size is %u x %u.\n%u bytes of work ares is free.\n", tjpg.jdec.width, tjpg.jdec.height, tjpg.jdec.sz_pool);
        res = tjpg.decomp(my_out_func, 0);
        if (res == JDR_OK) {
            printf("\rDecompression succeeded.\n");
        } else {
            printf("jd_decomp() failed (rc=%d)\n", res);
        }
    } else {
        printf("jd_prepare() failed (rc=%d)\n", res);
    }

    while (1) tight_loop_contents();

}

