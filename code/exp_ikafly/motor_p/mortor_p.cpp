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
#include "../../lib/imu/src/imu.h"
#include "../../lib/ikafly_pin.h"
#include "../../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../../lib/motor/src/umortor.h"


#define I2C i2c1
const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;
#define RAD2DEG 57.2958
#define I2C i2c1
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define goal_latitude
#define goal_longgitude

IMU imu(I2C);
nmeap_context_t nmea;
 nmeap_gga_t gga;


int main(void){

    stdio_init_all();
	sleep_ms(2000);
    uart_init(UART, 9600);
	i2c_init(I2C, 400*1000);

    int ch;
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

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

	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
    imu.init();
	imu.setDebugPrint(false);
    imu.calibration();
	float euler[3];

while(1){
        
        imu.update();
		imu.getAttEuler(euler);
        printf("%+03.2f %+03.2f %+03.2f\n", euler[0]*RAD2DEG, euler[1]*RAD2DEG, euler[2]*RAD2DEG);
		sleep_ms(20);
         if(uart_is_readable(UART)){
        ch = uart_getc(UART);
        nmeap_parse(&nmea, ch);
        }

		}
}
float arctan;
float torig[4];
float arg;

void mortor_control(float yaw){
float p;
arctan=atan2(goal_latitude-gga.latitude,goal_latitude);

torig[0]=cos(arctan);
torig[1]=sin(arctan);

torig[2]=cos(yaw);
torig[3]=sin(yaw);

arg=atan2(-torig[3]*torig[0]+torig[2]*torig[1],torig[2]*torig[0]+torig[3]*torig[1]);

p=arg*46.877;
forward(800-p,800+p)

}
