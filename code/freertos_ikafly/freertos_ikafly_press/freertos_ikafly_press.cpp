#include "FreeRTOSConfig.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"


#include "../../lib/press/src/uprs.h"

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

	float press_hpa, alt_cm,temp_degc;
	float alt_cm_old=0.0;//to compere alt
	int i=0; //to delete this task
	printf("Time(ms) Pressure(hPa) Altitude(m) Temperature(degC)\n");
	while (1) {
		press_hpa = prs.getPressHpa();
		alt_cm = prs.getAltM();
		temp_degc = prs.getTempDegC();

		printf("%d %4.2f %4.2f %2.1f\n", time_us_32() / 1000, press_hpa, alt_cm, temp_degc);

		if(alt_cm>=alt_cm_old+0.30){
			printf("up");
		}else if(alt_cm+0.30<=alt_cm_old){
			printf("fall");
			i++;
		}else{
			printf("stop");
			if(i>=5){
				printf("taskdelete");
				vTaskDelete(NULL);
			}
		}

		alt_cm_old=alt_cm;
		

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
