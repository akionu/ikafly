#include "lis3mdl.h"
#include "hardware/i2c.h"

int32_t lis3mdl_i2c_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
	// no function send more than 1 byte in lis3mdl_reg.c
	uint8_t addr = ((uint8_t)LIS3MDL_I2C_ADD_L>>1);
	uint8_t wbuf[2] = {reg, bufp[0]};
	int16_t ret = i2c_write_blocking((i2c_inst_t *)handle, addr, wbuf, len+1, false);

	if (ret != PICO_ERROR_GENERIC) return 0;
	else return -1;
}

int32_t lis3mdl_i2c_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	uint8_t addr = ((uint8_t)LIS3MDL_I2C_ADD_L>>1);
	i2c_write_blocking((i2c_inst_t *)handle, addr, &reg, 1, true);
	int16_t ret = i2c_read_blocking((i2c_inst_t *)handle, addr, bufp, len, false);

	if (ret != PICO_ERROR_GENERIC) return 0;
	else return -1;
}

void lis3mdl_delay(uint32_t ms) {
	sleep_ms(ms);
}

