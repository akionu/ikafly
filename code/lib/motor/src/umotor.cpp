#include "umotor.h"

Motor::Motor() {
}

// private:
uint8_t Motor::init(uint8_t pin) {
    uint8_t slice = pwm_gpio_to_slice_num(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_function((pin + 1), GPIO_FUNC_PWM);

    pwm_set_clkdiv(slice, 1.0614809); // clkdiv = sysclock / ((wrap + 1) * f)
    pwm_set_wrap(slice, 1023);        // pwm resolution = 1024, 115kHz
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    return slice;
}

void Motor::rotate(uint8_t slice, int16_t pwm) {
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

// public:
void Motor::init(uint8_t pin_left, uint8_t pin_right) {
    slice_left = init(pin_left);
    slice_right = init(pin_right);
    
    rotate(slice_left, 0);
   	rotate(slice_right, 0);
}

void Motor::forward(int16_t pwm) {
	forward(pwm, pwm);
}

void Motor::forward(int16_t pwm_left, int16_t pwm_right) {
    pwm_left_now = dir_left*pwm_left;
    pwm_right_now = dir_right*pwm_right;
    rotate(slice_left, pwm_left_now);
    rotate(slice_right, pwm_right_now);
}

void Motor::backward(int16_t pwm) {
	forward(-pwm);
}

void Motor::backward(int16_t pwm_left, int16_t pwm_right) {
	forward(-pwm_left, -pwm_right);
}

void Motor::stop() {
    rotate(slice_left, 0);
    rotate(slice_right, 0);
}

// 1: seiten, -1: gyakuten
void Motor::setDirForward(int8_t left, int8_t right) {
    int8_t ltmp, rtmp;
    ltmp = (left > 0) ? left : -left;
    rtmp = (right > 0) ? right : -right;
    if ((ltmp != 1) || (rtmp != 1)) {
        printf("err: setDirForward; left: %d, right: %d\n");
    }
    dir_left = left;
    dir_right = right;
}

int16_t Motor::getPwmLeft() {
    return (pwm_left_now);
}
int16_t Motor::getPwmRight() {
    return (pwm_right_now);
}
