#include "FreeRTOSConfig.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"


#include "../lib/press/src/uprs.h"

#define I2C i2c1

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;

Press prs(I2C, 0x77);


	void press_task(void *press){
	vTaskDelay(1000);

	i2c_init(I2C, 400*1000);
	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);

	prs.init();

	float press_hpa, alt_cm;
	float press_hpa_old=0.0;//to compere alt
	int i=0; //to delete this task
	printf("Time(ms) Pressure(hPa) Altitude(m)\n");
	while (1) {
		press_hpa = prs.getPressHpa();
		alt_cm = prs.getAltM();

		printf("%d %4.2f %4.2f \n", time_us_32() / 1000, press_hpa, alt_cm);

		if(press_hpa-press_hpa_old>=0.06){
			printf("fall");
            i++;
		}else if(press_hpa_old-press_hpa>=0.06){
			printf("up");
		}else{
			printf("stop");
			if(i>=5){
				printf("taskdelete");
				vTaskDelete(NULL);
			}
		}

		press_hpa_old=press_hpa;
		

		vTaskDelay(300);
	}

}


int main(void)
{
	stdio_init_all();
    xTaskCreate(press_task,"press_task",256,NULL,1,NULL);
    vTaskStartScheduler();
    while(1){};
    
}
