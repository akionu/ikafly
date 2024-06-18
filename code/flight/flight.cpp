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

#include "../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "semphr.h"


#include "../lib/imu/src/imu.h"
#include "../lib/ikafly_pin.h"
#include "../lib/rf/src/rf.h"
#include "../lib/motor/src/umotor.h"
#include "../lib/press/src/uprs.h"
#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"


#define I2C i2c1
#define RAD2DEG 57.2958
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define goal_latitude
#define goal_longitude




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
float latitude_ot;
float longitude_ot;






nmeap_context_t nmea;
nmeap_gga_t gga;
IMU imu(I2C);
Press prs(I2C, 0x77);
Motor motor;
Radio radio(24, 22);








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


void task_landing(void *landing){
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
		
		vTaskDelay(100);
	}
}









void task_get_imu(void *get_imu){
    stdio_init_all();
	sleep_ms(2000);
	i2c_init(I2C, 400*1000);

	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
    imu.init();
	imu.setDebugPrint(false);
    imu.calibration();
	float euler[3];

while(1){
        
        imu.update();
		imu.getAttEuler(euler);;
		yaw=euler[2];
		vTaskDelay(20);
		}
}

void task_send(void *send){
	radio.init();
    int latitude_digit=8;
	int longitude_digit=9;
    int i;
    float latitude_s;
	float longitude_s;


    while(1){
        if(gga.latitude !=0.0 && gga.latitude>=9){

        latitude_s=gga.latitude;
		longitude_s=gga.longitude;

        latitude_s=latitude_s*1000000;
		longitude_s=longitude_s*1000000;
        for(i=0;i<8;i++){  
            packet[i]=(int)latitude_s/(int)pow((double)10,(double)latitude_digit-i-1);
            latitude_s=(int)latitude_s%(int)pow((double)10,(double)latitude_digit-i-1);
            }
		for(i=8;i<=16;i++){
			packet[i]=(int)longitude_s/(int)pow((double)10,(double)longitude_digit-i+7);
            longitude_s=(int)longitude_s%(int)pow((double)10,(double)longitude_digit-i-1);
		}

            radio.send(packet);
            vTaskDelay(1000);
        }
        vTaskDelay(1000);
    }
}




void assemble_lat(uint8_t packet_r[32]){
	while(1){
  latitude_ot=0;
  longitude_ot=0;

  int i;
  for(i=0;i<=7;i++){
	latitude_ot=latitude_ot+packet_r[i]*pow((double)10,(double)latitude_digit-i-1);
  }
  for(i=8;i<=16;i++){
	longitude_ot=longitude_ot+packet_r[i]*pow((double)10,(double)longitude_digit-i-7);
  }
}


}

void task_recieve(void *recieve){



	while (1) {
		
		radio.receive(packet);
		for (int8_t i = 0; i < 32; i++) {
			printf("%x", packet[i]);
			assemble_lat(packet);
		}
		    printf("\n");
	    }


}








float distance(){
	double RX = 6378.137; 
    double RY = 6356.752; 

  double dx = goal_longitude - gga.longitude, dy = goal_latitude - gga.latitude;
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

        if(uart_is_readable(UART)){
        ch = uart_getc(UART);
        nmeap_parse(&nmea, ch);
		dis=distance();
        }
		
    }
}









void task_stack(void *stack){
	while(1){
	int i;
	float dis_av;
	float dis_sum=0;
	float dis_old[10];
		for(i=0;i<10;i++){
			dis_old[i]=dis;
			vTaskDelay(500);
		}
		for(i=0;i<9;i++){
			if(dis_old[i+1]-dis_old[i]>=0){
				dis_sum=dis_sum+dis_old[i+1]-dis_old[i];
			}else{
				dis_sum=dis_sum+dis_old[i]-dis_old[i+1];
			}
		}

		float yaw_sum=0;
		float yaw_av;
		float yaw_old[10];
		
		for(i=0;i<10;i++){
			yaw_old[i]=yaw;
			vTaskDelay(500);
		}
		for(i=0;i<9;i++){
			if(yaw_old[i+1]-yaw_old[i]>=0){
				yaw_sum=yaw_sum+yaw_old[i+1]-yaw_old[i];
			}else{
				yaw_sum=yaw_sum+yaw_old[i]-yaw_old[i+1];
			}
		}
	

		yaw_av=yaw_sum/9;
		dis_av=dis_sum/9;
		if(dis_av<=0.01 && yaw_av<=0.01){
			motor.forward(1023);
		}
	}

		

}









void task_g_motor_control(void *g_mortor_control){
float p;

while(1){

	if(dis<=2){
		vTaskDelete(NULL);
	}


	arctan=atan2(goal_latitude-gga.latitude,goal_latitude-gga.longitude);

	torig[0]=cos(arctan);
	torig[1]=sin(arctan);

	torig[2]=cos(yaw);
	torig[3]=sin(yaw);

	arg=atan2(-torig[3]*torig[0]+torig[2]*torig[1],torig[2]*torig[0]+torig[3]*torig[1]);

	p=arg*46.877;
	motor.forward(800-p,800+p);

}

}








int main(void){
    stdio_init_all();
    vTaskDelay(2000);
	stdio_usb_init();

	motor.init(pin_left_begin, pin_right_begin);

    SemaphoreHandle_t xMutex;
    xMutex = xSemaphoreCreateMutex();

    xTaskCreate(task_landing,"task_landing",256,NULL,6,NULL);
    xTaskCreate(task_get_gnss,"task_get_gnss",256,NULL,4,NULL);
    xTaskCreate(task_get_imu,"task_get_imu",256,NULL,3,NULL);
	xTaskCreate(task_stack,"task_stack",256,NULL,2,NULL);
	xTaskCreate(task_g_motor_control,"task_g_motor_control",256,NULL,1,NULL);
	xTaskCreate(task_send,"task_send",256,NULL,5,NULL);
    vTaskStartScheduler();
    while(1);
}