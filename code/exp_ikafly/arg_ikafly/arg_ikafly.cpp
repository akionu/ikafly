#include <cmath>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "FreeRTOSConfig.h"
#include "pico/stdio_usb.h"



#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "semphr.h"



#include "../../lib/imu/src/imu.h"
#include "../../lib/ikafly_pin.h"
#include "../../lib/rf/src/rf.h"
#include "../../lib/motor/src/umotor.h"
#include "../../lib/press/src/uprs.h"
#include "../../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"

#define I2C i2c1
#define RAD2DEG 57.2958
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define goal_latitude 40.2121
#define goal_longitude 140.0266



const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;
float dis;
float arctan;
float torig[4];
float arg;
float yaw;
uint8_t packet[32];
int latitude_digit=8;
int longitude_digit=9;
float latitude_ot=0;
float longitude_ot=0;
int cgg=0;
int cgg_o;
float p;



IMU imu(I2C);
Motor motor;
nmeap_context_t nmea;
nmeap_gga_t gga;



void task_get_imu(void *get_imu){

    TickType_t lastunblock_imu;
    lastunblock_imu=xTaskGetTickCount();

    imu.init();
	imu.setDebugPrint(false);
    //imu.calibration();
	float euler[3];

    printf("lll");

while(1){

        printf("jjj");

        imu.update();
		imu.getAttEuler(euler);


		yaw=euler[2];
		printf("imu");
		vTaskDelayUntil(&lastunblock_imu,pdMS_TO_TICKS(20));
        printf("ooo");
		}
}




static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
    cgg++;

}


void task_get_gnss(void *get_gnss){
    int ch;
    TickType_t lastunblock_gnss;
    lastunblock_gnss=xTaskGetTickCount();
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);


    uart_init(UART, 9600);
    gpio_set_function(MOSI, GPIO_FUNC_UART);
    gpio_set_function(MISO, GPIO_FUNC_UART);
    uart_set_baudrate(UART, 9600);
    uart_set_hw_flow(UART, false, false);
    uart_set_format(UART, 8, 1, UART_PARITY_NONE);


    uart_set_fifo_enabled(UART, true);
    irq_set_enabled(IRQ, true);
    uart_set_irq_enables(UART, true, false);


    nmeap_init(&nmea, NULL);
    nmeap_addParser(&nmea, "GNGGA", nmeap_gpgga, gpgga_callout, &gga);

    while (1)
    {	
            
			cgg_o=cgg;
			
			ch = uart_getc(UART);
      		nmeap_parse(&nmea, ch);

			if(cgg_o != cgg){
                printf("sus_a");
                vTaskDelayUntil(&lastunblock_gnss,pdMS_TO_TICKS(100));
				printf("sus");
			}

	}	
		
}




int main(){
    stdio_init_all();
    sleep_ms(2000);

	motor.init(pin_left_begin, pin_right_begin);

	i2c_init(I2C, 400*1000);

	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);

    
   
    xTaskCreate(task_get_gnss,"task_get_gnss",256,NULL,1,NULL);

    xTaskCreate(task_get_imu,"task_get_imu",256,NULL,2,NULL);

	
    vTaskStartScheduler();
    while(1){};
}
