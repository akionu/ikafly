#include "../src/motor.h"
#include "pico/platform.h"

#define PIN_L 18
#define PIN_R 20

int main() {
    stdio_init_all();

    uint8_t slice_left = motor_init(PIN_L);
    uint8_t slice_right = motor_init(PIN_R);

    int16_t pwm = 1023;
    motor_rotate(slice_left, -pwm);
    motor_rotate(slice_right, -pwm);

    while (1) {
        //if (pwm >= 1023) pwm = -1023;
        //pwm++;
        //sleep_ms(5);
		tight_loop_contents();
    }
}
