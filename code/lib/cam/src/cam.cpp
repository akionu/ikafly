#include "cam.h"

// public:
Cam::Cam(i2c_inst_t* i2c, uint8_t xclk, uint8_t y2_pio_base, PIO pio, uint8_t pio_sm, uint8_t dma_channel) {
	this->sccb 		      = i2c;
	this->pin_xclk        = xclk;
	this->pin_y2_pio_base = y2_pio_base;
	this->pio             = pio;
	this->pio_sm          = pio_sm;
	this->dma_channel     = dma_channel;
	this->image_buf_size  = sizeof(image_buf);
}


void Cam::init() {
	// XCLK generation
	gpio_set_function(pin_xclk, GPIO_FUNC_PWM);
	uint slice_num = pwm_gpio_to_slice_num(pin_xclk);

	// 6 cycles (0 to 5), 125 MHz / 6 = ~20.83 MHz wrap rate
	//	pwm_set_wrap(slice_num, 5);
	//	pwm_set_gpio_level(pin_xclk, 3);

	// 10 cylcles (0 to 9), 125MHz / 10 = 12.5MHz wrap rate
	pwm_set_wrap(slice_num, 9);
	pwm_set_gpio_level(pin_xclk, 5); // duty=50%

	// 20 cycles (0 to 19), 125MHz / 20 = 6.25MHz wrap rate
	//	pwm_set_wrap(slice_num, 19);
	//	pwm_set_gpio_level(pin_xclk, 10);

	pwm_set_enabled(slice_num, true);

	// Enable image RX PIO
	offset = pio_add_program(pio, &image_program);
	image_program_init(pio, pio_sm, offset, pin_y2_pio_base);
}

void Cam::enableJpeg() {
	
	reg_write(0xff, 0x01); // register bank 1
	reg_write(0x12, 0x80); // reset, cif

	regs_write(ov2640_jpeg_init_reg);
	regs_write(ov2640_yuv422);
	regs_write(ov2640_jpeg);
	
	reg_write(0xff, 0x01); // register bank 1
	reg_write(0x15, 0x00); // pin settings
	regs_write(ov2640_160_120_jpeg);
}

void Cam::capture() {

	// clear image buf
	for (uint16_t i = 0; i < image_buf_size; i++)
		image_buf[i] = 0x00;

	// DMA settings
	dma_channel_config c = dma_channel_get_default_config(dma_channel);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_dreq(&c, pio_get_dreq(pio, pio_sm, false));
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	
	dma_channel_configure(
		dma_channel, &c,
		image_buf,
		&pio->rxf[pio_sm],
		image_buf_size,
		false
	);

	// reset PIO's PC (maybe we need to overclock to do like picampinos)
	pio_sm_set_enabled(pio, pio_sm, false);
	image_program_init(pio, pio_sm, offset, pin_y2_pio_base);
	pio_sm_set_enabled(pio, pio_sm, true);

	dma_channel_start(dma_channel);
}

bool Cam::isCaptureFinished() {
	return (!dma_channel_is_busy(dma_channel));
}

uint16_t Cam::getJpegSize() {
	// search for the end marker of jpeg
	for (uint16_t i = 0; i < image_buf_size-1; i++) {
		if (image_buf[i] == 0xff &&
				image_buf[i+1] == 0xd9) {
			return (i+2);
		}
	}
	return 0; // no marker found(potentially broken image)
}

// private:
uint8_t Cam::reg_read(uint8_t reg) {
	
	i2c_write_blocking(sccb, OV2640_ADDR, &reg, 1, false);

	uint8_t value;
	i2c_read_blocking(sccb, OV2640_ADDR, &value, 1, false);

	return value;
}

void Cam::reg_write(uint8_t reg, uint8_t value) {
	uint8_t data[] = {reg, value};
	i2c_write_blocking(sccb, OV2640_ADDR, data, sizeof(data), false);
}

void Cam::regs_write(const uint8_t (*regs_list)[2]) {
	while (1) {
		uint8_t reg = (*regs_list)[0];
		uint8_t value = (*regs_list)[1];

		if (reg == 0x00 && value == 0x00) {
			break;
		}

		reg_write(reg, value);

		regs_list++;
	}
}


void Cam::image_program_init(PIO pio, uint sm, uint offset, uint pin_base) {
	pio_sm_set_consecutive_pindirs(pio, sm, pin_base, 11, false);

	pio_sm_config c = image_program_get_default_config(offset);
	sm_config_set_in_pins(&c, pin_base);
	sm_config_set_in_shift(&c, false, true, 8);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
	sm_config_set_clkdiv(&c, 1);

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
}
