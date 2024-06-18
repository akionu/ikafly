#pragma once

#include "hardware/i2c.h"
#include <stdint.h>

// Camera (DVP)
const uint8_t pin_dvp_exclk = 0;
const uint8_t pin_dvp_y2    = 1;

// UART (GNSS)
const uint8_t pin_gnss_vcc   = 23; // input LOW to ON
const uint8_t pin_uart0_mosi = 12;
const uint8_t pin_uart0_miso = 13;

// Sensors
i2c_inst_t* bus_i2c  = i2c1;
const uint8_t pin_i2c1_sda = 14;
const uint8_t pin_i2c1_scl = 15;

// Motor
const uint8_t pin_motor1_a = 18;
const uint8_t pin_motor1_b = 19;
const uint8_t pin_motor2_a = 20;
const uint8_t pin_motor2_b = 21;

// Radio
const uint8_t pin_rf_mosi = 24; // tx
const uint8_t pin_rf_miso = 22; // rx

// LED
const uint8_t pin_led = 29;

// Nichrome
const uint8_t pin_nichrome_left = 16;
const uint8_t pin_nichrome_right = 17;

const uint pin_left_begin = 18;
const uint pin_right_begin = 20;


