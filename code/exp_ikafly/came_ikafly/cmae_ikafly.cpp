#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "cam.h"
#include "tjpg.hpp"

#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"

#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "semphr.h"

#include "../../lib/motor/src/umotor.h"

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
Motor motor;

// 'Red' in HSV
// Hue
#define H_MIN_1 0 // 固定
#define H_MAX_1 60
#define H_MIN_2 300
#define H_MAX_2 360 // 固定
// Satuation
#define S_MIN 10
#define S_MAX 100
// Value
#define V_MIN 15
#define V_MAX 90

// util funcs
#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

// Result: 20x15
uint32_t red_area[15] = {0};
uint32_t cnt = 0;
uint32_t area_u;
unit32_t area_m1;
unit32_t area_m2;
uint32_t area_m3;
uint32_t area_b;

uint32_t area_sum;

uint8_t con[4];

static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect)
{
    // ref:https://qiita.com/yosihisa/items/c326e59ca5d65f35a181
    const uint8_t N_BPP = 3; // RGB888
    uint8_t *src;
    uint16_t x, y; //, bws;
    uint8_t r, g, b;
    double h = 0, s, v;
    uint32_t bitshift = 0;

    //    printf("\n");
    src = (uint8_t *)bitmap;
    //    bws = N_BPP * (rect->right - rect->left + 1);
    for (y = 0; y < (rect->bottom - rect->top + 1); y++)
    {
        for (x = 0; x < (rect->right - rect->left + 1); x += 1)
        {
            //            printf("y: %d\n", y);
            r = *(src + 3 * (y * (rect->right - rect->left + 1) + x));
            g = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 1);
            b = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 2);
            //            printf("(%03d,%03d,%03d),", r,g,b);
            double MAX = max((max(r, g)), b);
            double MIN = min((min(r, g)), b);
            v = MAX / 256 * 100;

            if (MAX == MIN)
            {
                h = 0;
                s = 0;
            }
            else
            {
                if (MAX == r)
                    h = 60.0 * (g - b) / (MAX - MIN) + 0;
                else if (MAX == g)
                    h = 60.0 * (b - r) / (MAX - MIN) + 120.0;
                else if (MAX == b)
                    h = 60.0 * (r - g) / (MAX - MIN) + 240.0;

                if (h > 360.0)
                    h = h - 360.0;
                else if (h < 0)
                    h = h + 360.0;
                s = (MAX - MIN) / MAX * 100.0;
            }

            if (h > 360.0)
                h -= 360;

            // 赤色の判定
            if ((h >= H_MIN_1 && h <= H_MAX_1) || (h >= H_MIN_2 && h <= H_MAX_2))
            {
                if ((s >= S_MIN && s <= S_MAX) && (v >= V_MIN && v <= V_MAX))
                {
                    //                    printf("red!: (rx,ry)=(%d,%d)\n", rect->left+x, rect->top+y);
                    bitshift = (rect->left + x);
                    red_area[rect->top + y] |= (1 << bitshift);
                }
            }
        }
    }

    return 1; // Continue to decompress
}

void came(void *p)
{
    TickType_t lastj;
    lastj = xTaskGetTickCount();

    while (1)
    {
        printf("%d: ", cnt++);
        cam.capture();
        while (!cam.isCaptureFinished())
            tight_loop_contents();
        uint32_t size = cam.getJpegSize();
        printf("last: ");
        for (uint32_t i = size - 10; i < size; i++)
            printf("%02x", cam.image_buf[i]);
        printf(", size: %d\n", size);

        memset(red_area, 0, sizeof(red_area));
        JRESULT res = tjpg.prepare(cam.image_buf, size);
        if (res == JDR_OK)
        {
            printf("Image size is %u x %u.\n%u bytes of work area is free.\n", tjpg.jdec.width, tjpg.jdec.height, tjpg.jdec.sz_pool);
            res = tjpg.decomp(my_out_func, 3);
            // 160x120 -> (1/2^3) -> 20x15
            if (res == JDR_OK)
            {
                printf("\rDecompression succeeded.\n");
                // print red_area (20x15)
                uint8_t bit;
                for (uint8_t y = 0; y < 15; y++)
                {
                    //                   std::cout << std::bitset<32>(red_area[y]) << std::endl;
                    for (int8_t i = 20 - 1; i >= 0; i--)
                    {
                        bit = (red_area[y] >> i) & 1;
                        printf("%u", bit);
                    }
                    printf("\n");
                }
            }
            else
            {
                printf("jd_decomp() failed (rc=%d)\n", res);
            }
        }
        else
        {
            printf("jd_prepare() failed (rc=%d)\n", res);
        }

        area_u = red_area[0] && red_area[1] && red_area[2];
        area_m1 = red_area[3] && red_area[4] && red_area[5];
        area_m2 = red_area[6] && red_area[7] && red_area[8];
        area_m3 = red_area[9] && red_area[10] && red_area[11];
        area_b = red_area[12] && red_area[13] && red_area[14];

        area_sum = area_u || area_m1 || area_m2 || area_m3 || area_b;

        vTaskDelayUntil(&lastj, pdMS_TO_TICKS(100));
    }
}

void motor(void *p)
{

    while (1)
    {
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);

    i2c_init(I2C, 400 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);

    cam.init();
    cam.enableJpeg();

    motor.init(pin_left_begin, pin_right_begin);

    xTaskCreate(came, "cama", 256, NULL, 1, NULL);
    xTaskCreate(motor, "motor", 256, NULL, 2, NULL);
}