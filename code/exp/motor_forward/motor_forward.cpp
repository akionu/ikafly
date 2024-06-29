#include <stdio.h>
#include "pico/platform.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "../../lib/motor/src/umotor.h"

const uint pin_left_begin = 18;
const uint pin_right_begin = 20;

Motor motor;

int main(void) {
	stdio_usb_init();
	sleep_ms(1000);

	// モーターの初期化
	motor.init(pin_left_begin, pin_right_begin);

	// 回転方向の補正
	// 正回転させようとしたときに正回転したら1
	// 逆回転したら-1を左右に入れる
	motor.setDirForward(-1, 1);
	
	// 正回転（両方とも）
	// 引数は回転速度pwm（0<=pwm<=1023）
	motor.forward(1023);

	while (1) {
		tight_loop_contents();
	}
	
}
