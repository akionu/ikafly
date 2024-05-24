#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "ov2640_init.h"
#include "image.pio.h"

class Cam {
	public:
		Cam(i2c_inst_t* i2c = i2c1, uint8_t xclk = 0, uint8_t y2_pio_base = 1, PIO pio = pio0, uint8_t pio_sm = 0, uint8_t dma_channel = 0);
		void init(); // must initialize I2C bus before call this
		void enableJpeg();
		void capture();
		uint16_t getJpegSize();
		bool isCaptureFinished();
		uint8_t image_buf[1024*2+512]; // 2.5kB should be enough

	private:
		uint8_t reg_read(uint8_t reg);
		void reg_write(uint8_t reg, uint8_t value);
		void regs_write(const uint8_t (*regs_list)[2]);
		void image_program_init(PIO pio, uint sm, uint offset, uint pin_base);

		i2c_inst_t *sccb;
		uint8_t pin_sioc;
		uint8_t pin_siod;

		uint8_t pin_xclk;
		// Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, VSYNC, HREF< PCLK
		uint8_t pin_y2_pio_base;

		PIO pio;
		uint8_t pio_sm;

		uint8_t dma_channel;
		uint16_t image_buf_size;
		const uint8_t OV2640_ADDR = 0x60 >> 1;
		uint8_t offset = 0;
};
