/*#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../src/cam.h"

#define SDA 14
#define SCL 15
#define I2C i2c1
#define XCLK 0
#define Y2_PIO_BASE 1
#define PIO pio0
#define PIO_SM 0
#define DMA_CH 0

//Cam::Cam(i2c_inst_t* i2c, uint8_t xclk, uint8_t y2_pio_base, PIO pio, uint8_t pio_sm, uint8_t dma_channel) {
Cam cam(I2C, XCLK, Y2_PIO_BASE, PIO, PIO_SM, DMA_CH);
int main(void) {
	stdio_init_all();
	sleep_ms(2000);

	i2c_init(I2C, 400*1000);
	gpio_set_function(SDA, GPIO_FUNC_I2C);
	gpio_set_function(SCL, GPIO_FUNC_I2C);

	cam.init();
	cam.enableJpeg();

	while(1){
		cam.capture();
		while (!cam.isCaptureFinished()) tight_loop_contents();

		for (uint16_t i = 0; i < cam.getJpegSize(); i++)
			printf("%02x", cam.image_buf[i]);

		for (uint16_t i = 0; i < 64; i++) 
			printf("00");

		sleep_ms(1500);
	}
}
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "../../lib/cam/src/jpeg-9f/jpeglib.h"
#include "../../lib/cam/src/libbmp-master/libbmp.h"
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../lib/cam/src/cam.h"

#define SDA 14
#define SCL 15
#define I2C i2c1
#define XCLK 0
#define Y2_PIO_BASE 1
#define PIO pio0
#define PIO_SM 0
#define DMA_CH 0

Cam cam(I2C, XCLK, Y2_PIO_BASE, PIO, PIO_SM, DMA_CH);

int main(){
	struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);
}