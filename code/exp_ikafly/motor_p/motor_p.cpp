#include <cmath>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdio.h>

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





nmeap_context_t nmea;
nmeap_gga_t gga;
IMU imu(I2C);
Press prs(I2C, 0x77);
Motor motor;
Radio radio(24, 22);


float distance(float longitude, float latitude ,float longitude_self, float latitude_self){
	double RX = 6378.137; 
    double RY = 6356.752; 

  double dx = longitude - longitude_self, dy = latitude - latitude_self;
  double mu = (goal_latitude +gga.latitude ) / 2.0; 
  double E = sqrt(1 - pow(RY / RX, 2.0));
  double W = sqrt(1 - pow(E * sin(mu), 2.0));
  double M = RX * (1 - pow(E, 2.0)) / pow(W, 3.0)/pow(W,3.0);
  double N = RX / W;
  return sqrt(pow(M * dy, 2.0) + pow(N * dx * cos(mu), 2.0)); 
}

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
	cgg++;

}




void task_get_gnss(void *get_gnss){
    int ch;
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
                if(gga.latitude !=0 && gga.longitude !=0){
                dis=distance(gga.longitude,gga.latitude,goal_longitude,goal_latitude);
				vTaskDelay(100);
                }

				printf("sus");
			}


	}	
		
}



void task_get_imu(void *get_imu){

    imu.init();
	imu.setDebugPrint(false);
    imu.calibration();

	vTaskSuspend(NULL);
	float euler[3];

while(1){

        
        imu.update();
		imu.getAttEuler(euler);


		yaw=euler[2];
		printf("imu");
		vTaskDelay(20);
		}
}

void task_control_motor(void *cm){
float p;
while(1){

    if(gga.latitude !=0 && gga.longitude !=0){
arctan=atan2(goal_latitude-gga.latitude,goal_longitude-gga.longitude);

torig[0]=cos(arctan);
torig[1]=sin(arctan);

torig[2]=cos(yaw);
torig[3]=sin(yaw);

arg=atan2(-torig[3]*torig[0]+torig[2]*torig[1],torig[2]*torig[0]+torig[3]*torig[1]);

p=arg*46.877;
motor.forward(800-p,800+p);
vTaskDelay(100);
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
  xTaskCreate(task_control_motor,"task_control_motor",256,NULL,3,NULL);
  vTaskStartScheduler();
  while(1);




}
