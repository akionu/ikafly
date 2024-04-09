#include "motor.h"

uint8_t motor_init(uint8_t pin) {
    uint8_t slice = pwm_gpio_to_slice_num(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_function((pin + 1), GPIO_FUNC_PWM);

    pwm_set_clkdiv(slice, 1.0614809); // clkdiv = sysclock / ((wrap + 1) * f)
    pwm_set_wrap(slice, 1023);        // pwm resolution = 1024, 115kHz
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    return slice;
}

void motor_brake(uint8_t slice) {
    pwm_set_chan_level(slice, PWM_CHAN_A, 1023);
    pwm_set_chan_level(slice, PWM_CHAN_B, 1023);
    pwm_set_enabled(slice, true);
}

void motor_rotate(uint8_t slice, int16_t pwm) {
    // abs(pwm) < 500 (at 7.4V) means no rotation because of high(1:380) gear rates
    if (pwm > 0) {
        // forward
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, pwm);
    } else {
        // backward
        pwm_set_chan_level(slice, PWM_CHAN_A, -pwm);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    }
    pwm_set_enabled(slice, true);
}

