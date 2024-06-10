#include "FreeRTOSConfig.h"
#include "../../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"


#include "../../../lib/press/src/uprs.h"

#define I2C i2c1

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;

Press prs(I2C, 0x77);


void press_task(void *){
	vTaskDelay(1000);

	i2c_init(I2C, 400*1000);
	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);

	prs.init();
	float alt_cm,alt_con[50],alt_sum,alt_av,alt_old;
	int i;
	alt_sum=0.0;
	// decideing ground altitude = alt_av;
	printf("start geting avereg of altitude\n");


	for(i=0;i<=50;i++){
	alt_con[i]=prs.getAltM();
	printf("%d\n",i);
	vTaskDelay(100);
	};


	for(i=3;i<=50;i++){
		alt_sum=alt_sum+alt_con[i];
		printf("%d\n",i);
	};


	alt_av=alt_sum/48;
	printf("%f\n",alt_sum);
	printf("%f\n",alt_av);
	alt_old=alt_av;
	printf("done geting altitude\n");
	while(1){
		alt_cm=prs.getAltM();
		if(alt_cm-alt_av<=5 && alt_av-alt_cm<=5){
			printf("ground\n");
		}else if(alt_cm-alt_old>=1.0){
			printf("up\n");
		}else if(alt_old-alt_cm>=1.0){
			printf("down\n");
		}else {
			printf("hovering\n");
		};
		printf("%f",alt_cm);
		alt_old=alt_cm;
		vTaskDelay(100);
	}
	}


int main(void)
{
	stdio_init_all();
    xTaskCreate(press_task,"press_task",256,NULL,1,NULL);
    vTaskStartScheduler();
    while(1){};
    
}