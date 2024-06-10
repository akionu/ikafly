#include "FreeRTOSConfig.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/task.h"
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
#include "nmeap-0.3/inc/nmeap.h"
#include "../lib/imu/src/imu.h"
#include "../lib/press/src/uprs.h"
#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../lib/ikafly_pin.h"

#define I2C i2c1
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define RAD2DEG 57.2958
#define goal_long
#define goal_lat


IMU imu(I2C);

Press prs(I2C, 0x77);
const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;

 nmeap_context_t nmea;
 nmeap_gga_t gga;

float euler_yaw;
float distance_g;

//init gpio
void motor_init(){
	gpio_init(pin_motor1_a);
	gpio_init(pin_motor1_b);
	gpio_init(pin_motor2_a);
	gpio_init(pin_motor2_b);

	gpio_set_dir(pin_motor1_a,0);
	gpio_set_dir(pin_motor1_b,0);
	gpio_set_dir(pin_motor2_a,0);
	gpio_set_dir(pin_motor2_b,0);
}

void motor_foward(){
	gpio_put(pin_motor1_a,1);
	gpio_put(pin_motor1_b,0);
	gpio_put(pin_motor2_a,1);
	gpio_put(pin_motor2_b,0);
}

void motor_back(){
	gpio_put(pin_motor1_a,0);
	gpio_put(pin_motor1_b,1);
	gpio_put(pin_motor2_a,0);
	gpio_put(pin_motor2_b,1);
}

void motor_r_rotate(){
	gpio_put(pin_motor1_a,0);
	gpio_put(pin_motor1_b,1);
	gpio_put(pin_motor2_a,0);
	gpio_put(pin_motor2_b,0);
}

void motor_l_rotate(){
	gpio_put(pin_motor1_a,0);
	gpio_put(pin_motor1_b,0);
	gpio_put(pin_motor2_a,1);
	gpio_put(pin_motor2_b,0);
}

void nichrom(){

}


void task_landing(void *landing){
    int p;
	gpio_init(pin_nichrome_right);
	gpio_init(pin_nichrome_left);

	prs.init();
	float alt_cm,alt_con[52],alt_av,alt_old;
	int i;
	for(i=0;i<=50;i++){
	alt_con[i]=prs.getAltM();
	vTaskDelay(100);
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
				vTaskDelete(xTaskGetCurrentTaskHandle());
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
		
		vTaskDelay(100);
	}
	}

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
}

void task_gnss(void *gnss)
{
    int ch;
	float euler[3];
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
        ch = uart_getc(UART);
        nmeap_parse(&nmea, ch);
		printf("%f,%f",gga.latitude,gga.longitude);
        
    }
}

void task_imu(void *get_imu){

	imu.setDebugPrint(false);
  	imu.init();
	float euler[3];
while(1){
		imu.update();
		imu.getAttEuler(euler);
		vTaskDelay(49);
		euler_yaw=euler[2];
}
}

float calc_distance_g(){
	double RX = 6378.137; 
double RY = 6356.752; 

  double dx = goal_long - gga.longitude, dy = goal_lat - gga.latitude;
  double mu = (goal_lat +gga.latitude ) / 2.0; 
  double E = sqrt(1 - pow(RY / RX, 2.0));
  double W = sqrt(1 - pow(E * sin(mu), 2.0));
  double M = RX * (1 - pow(E, 2.0)) / pow(W, 3.0)/pow(W,3.0);
  double N = RX / W;
  return sqrt(pow(M * dy, 2.0) + pow(N * dx * cos(mu), 2.0)); 
}




void task_gnss_control(void *gc){
	float arctan;
	distance_g=calc_distance_g()*1000;
while(1){
	arctan=atan2(gga.latitude,gga.longitude);

	if(distance_g<=2.0){vTaskSuspend(xTaskGetCurrentTaskHandle());
	}else if(euler_yaw-arctan<=10 || arctan-euler_yaw<=10){
		motor_foward();
		vTaskDelay(5000);
	};

}


}

int main(void)
{

	stdio_init_all();
	i2c_init(I2C, 400*1000);
	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
	motor_init();



    xTaskCreate(task_landing,"task_landing",256,NULL,2,NULL);
	xTaskCreate(task_gnss,"task_gnss",256,NULL,1,NULL);
	xTaskCreate(task_imu,"task_imu",256,NULL,3,NULL);
	xTaskCreate(task_gnss_control,"task_gnss_control",256,NULL,4,NULL);

    vTaskStartScheduler();
    while(1){};
    
}

