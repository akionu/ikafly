#pragma once

#include "./lsm6dso-pid/lsm6dso_reg.h"
#include "hardware/i2c.h"

int32_t lsm6dso_i2c_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t lsm6dso_i2c_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void lsm6dso_delay(uint32_t ms);

