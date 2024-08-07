#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

class Motor {
    public:
        Motor();
		void init(uint8_t pin_left, uint8_t pin_right);
        void forward(int16_t pwm);
        void forward(int16_t pwm_left, int16_t pwm_right);
        void backward(int16_t pwm);
        void backward(int16_t pwm_left, int16_t pwm_right);
        void stop();
        void setDirForward(int8_t left, int8_t right);
        int16_t getPwmLeft();
        int16_t getPwmRight();
    private:
		uint8_t init(uint8_t pin);
		void rotate(uint8_t slice, int16_t pwm);
        uint8_t slice_left;
        uint8_t slice_right;
        bool is_dir_set;
        int8_t dir_left;
        int8_t dir_right;
        int16_t pwm_left_now, pwm_right_now;
};

