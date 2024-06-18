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

//配列

float arctan;
float g_sin;
float g_cos;
float ika_sin;
float ika_cos;
float arg;



IMU imu(I2C);
nmeap_context_t nmea;
 nmeap_gga_t gga;

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
    nmeap_gga_t *gga = (nmeap_gga_t *)data;
}




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

void motor_stop(){

    gpio_put(pin_motor1_a,0);
	gpio_put(pin_motor1_b,0);
	gpio_put(pin_motor2_a,0);
	gpio_put(pin_motor2_b,0);

}

void motor_foward(){
	gpio_put(pin_motor1_a,0);
	gpio_put(pin_motor1_b,0);
	gpio_put(pin_motor2_a,0);
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

void argmatch(){
    while(g_cos-ika_cos>=0.3 || ika_cos-g_cos>=0.3){
    }
    motor_stop();
}

float motor1_control(float yaw){
    
    arctan=atan2(goal_latitude-gga.latitude,goal_longgitude-gga.longitude);
    g_sin=sin(arctan);
    g_cos=cos(arctan);
    ika_sin=sin(yaw);
    ika_cos=cos(yaw);

    if(ika_sin*g_sin>=0){
        arg=g_cos-ika_cos;

        if(arg>=0){
                motor_r_rotate();
                argmatch();

        }else{
                motor_l_rotate();
        }       argmatch();

    }else if(g_cos*ika_cos>=0){
        if(g_cos>=0){
            if(g_sin>=0){
                motor_l_rotate();
                argmatch();
            }else{
                motor_r_rotate();
                argmatch();
            }
        }else{
            if(g_sin>=0){
                motor_r_rotate();
                argmatch();
            }else{
                motor_l_rotate();
                argmatch();
            }
        }
    }else{
        if(g_sin>=0.7071 && ika_sin>=-0.7071 ){
            motor_r_rotate();
            argmatch();
        }else if(g_sin<=0.7071 && ika_sin<=-0.7071){
            motor_r_rotate();
            argmatch();
        }else{
            motor_r_rotate();
            argmatch();
        }
    }


}




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
        motor1_control(euler[2]);
		}
}

