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


#define I2C i2c1

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;

Press prs(I2C, 0x77);


	void task_press(void *press){
	vTaskDelay(1000);

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

#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13

static nmeap_context_t nmea;
static nmeap_gga_t gga;

static void print_gga(nmeap_gga_t *gga)
{
    printf("%.6f,%.6f\n",gga->latitude,gga->longitude);
}


/** called when a gpgga message is received and parsed */

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
}

void task_gps(void *gps)
{
    int ch;
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

    // uart setting
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
        print_gga(&gga);
    }
}

#define RAD2DEG 57.2958

IMU imu(I2C);

void task_imu(void *){
	vTaskDelay(2000);
	printf("This is imu_euler\n");
	
	imu.setDebugPrint(false);

	bool st = imu.init();
	if (st) printf("IMU init success");
	else printf("IMU init fails");

	float euler[3];//, accel[3], gyro[3], mag[3];
	printf("roll pitch yaw\n");
	while (1) {
		imu.update(); // should call this 50ms each(20Hz), refer MadgwickAHRS.c and sensor output frequency in init() in imu.cpp
		//imu.getAccel_mg(accel);
		//printf("%3.2f %3.2f %3.2f\n", accel[0], accel[1], accel[2]);
		imu.getAttEuler(euler);
		printf("%+03.2f %+03.2f %+03.2f\n", euler[0]*RAD2DEG, euler[1]*RAD2DEG, euler[2]*RAD2DEG);
		vTaskDelay(49);
	}
}

int main(void)
{

	stdio_init_all();
	i2c_init(I2C, 400*1000);
	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);

    xTaskCreate(task_press,"task_press",256,NULL,3,NULL);
	xTaskCreate(task_gps,"task_gps",256,NULL,1,NULL);
	xTaskCreate(task_imu,"task_imu",256,NULL,2,NULL);
    vTaskStartScheduler();
    while(1){};
    
}