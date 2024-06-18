#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "../../lib/press/src/uprs.h"
#define pin_nichrome_left 16
#define pin_nichrome_right 17


#define I2C i2c1
Press prs(I2C, 0x77);

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;

void nichrom(){
	gpio_init(pin_nichrome_left);
	gpio_init(pin_nichrome_right);

	gpio_set_dir(pin_nichrome_left,0);
	gpio_set_dir(pin_nichrome_right,0);

	gpio_put(pin_nichrome_left,1);
	gpio_put(pin_nichrome_right,1);

	sleep_ms(500);

	gpio_put(pin_nichrome_left,0);
	gpio_put(pin_nichrome_right,0);
}

int main(void){
	stdio_init_all();
	sleep_ms(1000);


	i2c_init(I2C, 400*1000);
	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);



    int p;
	prs.init();
	float alt_m,alt_con[52],alt_av,alt_old;
	int i;
	printf("ooo");
	for(i=0;i<=50;i++){
	alt_con[i]=prs.getAltM();
	sleep_ms(100);
	};
	for(i=10;i<=50;i++){
		alt_con[51]+=alt_con[i];
	};

	alt_av=alt_con[51]/41;
	printf("av,%f\n",alt_av);
	alt_old=alt_av;
	printf("done geting altitude");
	while(1){
		alt_m=prs.getAltM();
		if(alt_m-alt_av<=0.01 && alt_av-alt_m<=0.01){
			printf("ground");
			printf("%f",alt_m);
			if(p>=10){
				nichrom();
				
			}
		}else if(alt_m-alt_old>=0.01){
			printf("up");
		}else if(alt_old-alt_m>=0.01){
			printf("down");
		}else {
			printf("hovering");
		};
		alt_old=alt_m;
	
		if(alt_m-alt_av>=50){
			p++;
		};
		
		sleep_ms(500);
	}
	}