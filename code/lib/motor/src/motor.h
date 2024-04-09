#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

/// @brief initialize pwm channels for motor
/// @param pin gpio num. pin and pin+1 will be initialied
/// @return slice number. must handle this to rotate/stop motor
uint8_t motor_init(uint8_t pin);

/// @brief rotate motor
/// @param slice slice number to be manipulated, ret value of motor_init
/// @param pwm -1023 to 1023. - counterclockwise, 0 stop, + clockwise
void motor_rotate(uint8_t slice, int16_t pwm);

void motor_brake(uint8_t slice);

