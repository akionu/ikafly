#pragma once

#include "./lis3mdl-pid/lis3mdl_reg.h"
#include "hardware/i2c.h"

int32_t lis3mdl_i2c_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t lis3mdl_i2c_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void lis3mdl_delay(uint32_t ms);

