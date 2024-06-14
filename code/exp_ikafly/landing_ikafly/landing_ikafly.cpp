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
#include "../../lib/ikafly_pin.h"

#define I2C i2c1
Press prs(I2C, 0x77);

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
    int p;

	prs.init();
	float alt_cm,alt_con[52],alt_av,alt_old;
	int i;
	for(i=0;i<=50;i++){
	alt_con[i]=prs.getAltM();
	sleep_ms(100);
	};
	for(i=0;i<=50;i++){
		alt_con[51]+=alt_con[i];
	};

	alt_av=alt_con[51]/50;
	alt_old=alt_av;
	printf("done geting altitude");
	while(1){
		alt_cm=prs.getAltM();
		if(alt_cm-alt_av<=5 || alt_av-alt_cm<=5){
			printf("ground");
			if(p>=10){
				nichrom();
		
			}
		}else if(alt_cm-alt_old>=3.0){
			printf("up");
		}else if(alt_old-alt_cm>=3.0){
			printf("down");
		}else {
			printf("hovering");
		};
		alt_old=alt_cm;
	
		if(alt_cm-alt_av>=50){
			p++;
		};
		
		sleep_ms(100);
	}
	}