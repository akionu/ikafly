#include <cmath>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "pico/error.h"
#include "pico/flash.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

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
#define RAD2DEG 57.2958
#define pai 3.1415
#define UART uart0
#define IRQ UART0_IRQ
#define MOSI 12
#define MISO 13
#define XCLK 0
#define Y2_PIO_BASE 1
#define PIO pio0
#define PIO_SM 0
#define DMA_CH 0
#define goal_latitude 34.801665
#define goal_longitude 135.771090

// Hue
#define H_MIN_1 0 // 固定
#define H_MAX_1 60
#define H_MIN_2 300
#define H_MAX_2 360 // 固定
// Satuation
#define S_MIN 10
#define S_MAX 100
// Value
#define V_MIN 15
#define V_MAX 90

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

const uint i2c_sda_pin = 14;
const uint i2c_scl_pin = 15;
double dis;
double arctan;	 // argument between ikafly`s direction and goal
double torig[4]; // Trigonometric function
double arg;		 // argument between ikafly`s direction and goal
double yaw;
float alt_cm;
uint8_t packet[32] = {0};
uint8_t packet_m[32] = {0}; // be sent to other ikafly
uint32_t latitude_ot = 0;	// other ikafly`s latitude
uint32_t longitude_ot = 0;	// othe ikafly`s longitude
uint32_t gga_s[3];
uint32_t log_w[8] = {0};
uint32_t red_area[15] = {0};
int8_t cgg = 0; // increment when gga is resdable
int8_t cgg_o = 0;
int stacking = 0; // when ikafly is tacking, this is changed 1
int landed = 0;

nmeap_context_t nmea;
nmeap_gga_t gga;
IMU imu(I2C);
Press prs(I2C, 0x77);


	void task_press(void *press){
	vTaskDelay(1000);

	prs.init();

	float alt_con[52], alt_av, alt_old;
	int i;
	// get average altitude of ground
	for (i = 0; i <= 50; i++)
	{
		alt_con[i] = prs.getAltM();
		vTaskDelay(100);
	};
	for (i = 10; i <= 50; i++)
	{
		alt_con[51] += alt_con[i];
	};

	alt_av = alt_con[51] / 41;
	alt_old = alt_av;

	printf("done geting altitude");
	vTaskResume(get_imu_h);
	vTaskSuspend(NULL);

	while (1)
	{
		alt_cm = prs.getAltM();
		xQueueSend(press_q, &alt_cm, 0);
		if (alt_cm - alt_av <= 0.1 && alt_av - alt_cm <= 0.1)
		{
			printf("ground");
			if (up_s >= 10)
			{
				nichrom();

				landed = 1;

				vTaskResume(get_imu_h);
				vTaskResume(g_motor_control_h);
				vTaskResume(receive_h);
				vTaskResume(send_h);
				printf("landing_delete");
				vTaskDelete(NULL);
			}
		}
		else if (alt_cm - alt_old > 0.1)
		{
			printf("up");
		}
		else if (alt_old - alt_cm > 0.1)
		{
			printf("down");
		}
		else
		{
			printf("hovering");
		}
		alt_old = alt_cm;

		up_s++;

		if (up_s >= 10)
		{
			nichrom();
			printf("landing_delete");
			vTaskResume(g_motor_control_h);
			vTaskResume(receive_h);
			vTaskResume(send_h);
			vTaskResume(get_imu_h);
			vTaskDelete(NULL);
		}
		vTaskDelayUntil(&lastunlock_lan, pdMS_TO_TICKS(100));
		// vTaskDelay(100);
	}
}

void task_get_imu(void *get_imu)
{
	vTaskSuspend(NULL);

	imu.init();
	imu.setDebugPrint(false);
	// imu.calibration();

	TickType_t lastunblock_imu;
	lastunblock_imu = xTaskGetTickCount();
	float euler[3];

	imu_q = xQueueCreate(1,sizeof(euler[2]));

	vTaskResume(get_gnss_h);
	vTaskSuspend(NULL);

	while (1)
	{

		imu.update();
		imu.getAttEuler(euler);

		yaw = euler[2];
		xQueueSend(imu_q, &euler[2], 0);
			vTaskDelayUntil(&lastunblock_imu, pdMS_TO_TICKS(20));
	}
}

void assemble_lat(uint8_t packet_r[32])
{
	int i;

	while (1)
	{
		latitude_ot = 0;
		longitude_ot = 0;

		for (i = 0; i < 4; i++)
		{
			latitude_ot = latitude_ot | packet_r[i];
			if (i != 3)
			{
				latitude_ot = latitude_ot << 8;
			}
		}
		for (i = 0; i < 4; i++)
		{
			longitude_ot = longitude_ot | packet_r[i + 4];
			if (i != 3)
			{
				longitude_ot = longitude_ot << 8;
			}
		}
	}
}

void task_receieve(void *receieve)
{

	TickType_t lastunblock_receive;
	lastunblock_receive = xTaskGetTickCount();
	vTaskSuspend(NULL);

	while (1)
	{

		radio.receive(packet_m);
		printf("receive");

		vTaskSuspend(send_h);

		for (int k = 0; k < 5; k++)
		{
			radio.receive(packet_m);
		}
		vTaskResume(send_h);
		assemble_lat(packet_m);

		printf("\n");
		vTaskDelayUntil(&lastunblock_receive, pdMS_TO_TICKS(4000));
	}
}

void task_send(void *p)
{

	int i;
	uint32_t gga_ss[3];
	TickType_t ltusend;
	ltusend = xTaskGetTickCount();
	vTaskSuspend(NULL);

	while (1)
	{
		xQueueReceive(gnss_q, &gga_ss, 10);

		for (i = 0; i < 4; i++)
		{
			packet[i] = gga_ss[0] >> 8 * (3 - i);
		}
		for (i = 0; i < 4; i++)
		{
			packet[i + 4] = gga_ss[0] >> 8 * (3 - i);
		}
		// packet[8] = stacking;

		printf("sending\n");
		radio.send(packet);

		printf("j");
		vTaskDelayUntil(&ltusend, pdMS_TO_TICKS(4000));
		printf("k");
	}
}

// calculate distsns between ikafly and goal (and other ikafly)
float distance(double longitude, double latitude, double longitude_self, double latitude_self)
{
	double RX = 6378.137;
	double RY = 6356.752;

	double dx = longitude - longitude_self, dy = latitude - latitude_self;
	double mu = (goal_latitude + gga.latitude) / 2.0;
	double E = sqrt(1 - pow(RY / RX, 2.0));
	double W = sqrt(1 - pow(E * sin(mu), 2.0));
	double M = RX * (1 - pow(E, 2.0)) / pow(W, 3.0) / pow(W, 3.0);
	double N = RX / W;
	return sqrt(pow(M * dy, 2.0) + pow(N * dx * cos(mu), 2.0));
}

static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data)
{
	nmeap_gga_t *gga = (nmeap_gga_t *)data;
	cgg++;
	if (cgg == 260)
	{
		cgg = 0;
	}
}

void task_get_gnss(void *get_gnss)
{
	vTaskSuspend(NULL);

	int j = 0;
	uint8_t ch;
	TickType_t lastunblock_gnss;
	lastunblock_gnss = xTaskGetTickCount();

	gnss_q = xQueueCreate(1, sizeof(gga_s));

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

		cgg_o = cgg;

		ch = uart_getc(UART);
		nmeap_parse(&nmea, ch);

		if (cgg_o != cgg)
		{
			if (gga.latitude != 0 && gga.longitude != 0)
			{
				if (j == 0)
				{
					printf("vv");
					j++;
					vTaskResume(landing_h);
					// vTaskResume(log_h);
				}

				dis = distance(goal_longitude, goal_latitude, gga.longitude, gga.latitude);

				gga_s[0] = gga.latitude * 10000000;
				gga_s[1] = gga.longitude * 10000000;
				gga_s[2] = dis;

				xQueueSend(gnss_q, &gga_s, 0);
			}
			vTaskDelayUntil(&lastunblock_gnss, pdMS_TO_TICKS(1000));
			// vTaskDelay(100);
		}
	}
}

void task_stack(void *stack)
{
	vTaskSuspend(NULL);
	TickType_t lastunblock_stack;
	lastunblock_stack = xTaskGetTickCount();

	while (1)
	{
		int i;
		float dis_av;
		float dis_sum = 0;
		float dis_old[10];
		for (i = 0; i < 10; i++)
		{
			dis_old[i] = dis;
			vTaskDelayUntil(&lastunblock_stack, pdMS_TO_TICKS(100));
		}
		for (i = 0; i < 9; i++)
		{
			if (dis_old[i + 1] - dis_old[i] >= 0)
			{
				dis_sum = dis_sum + dis_old[i + 1] - dis_old[i];
			}
			else
			{
				dis_sum = dis_sum + dis_old[i] - dis_old[i + 1];
			}
		}

		float yaw_sum = 0;
		float yaw_av;
		float yaw_old[10];

		for (i = 0; i < 10; i++)
		{
			yaw_old[i] = yaw;
			vTaskDelayUntil(&lastunblock_stack, pdMS_TO_TICKS(500));
		}
		for (i = 0; i < 9; i++)
		{
			if (yaw_old[i + 1] - yaw_old[i] >= 0)
			{
				yaw_sum = yaw_sum + yaw_old[i + 1] - yaw_old[i];
			}
			else
			{
				yaw_sum = yaw_sum + yaw_old[i] - yaw_old[i + 1];
			}
		}

		yaw_av = yaw_sum / 9;
		dis_av = dis_sum / 9;

		if (dis_av <= 0.01 && yaw_av <= 0.01)
		{
			stacking = 1;
			motor.forward(1023);
		}
		else
		{
			stacking = 0;
		}
		vTaskDelayUntil(&lastunblock_stack, pdMS_TO_TICKS(1000));
	}
}

void task_g_motor_control(void *g_mortor_control)
{

	vTaskSuspend(NULL);
	printf("g_motor");
	float p;
	int j = 0;
	float dis_ot = 100;
	float dis_ot_g;

	TickType_t lastunblock_gmotor;
	lastunblock_gmotor = xTaskGetTickCount();

	while (1)
	{
		printf("move");

		// calculate argument between ikafly and goal
		arctan = atan2(goal_longitude - gga.longitude, goal_latitude - gga.latitude);

		torig[0] = cos(arctan);
		torig[1] = sin(arctan);

		torig[2] = cos(yaw);
		torig[3] = sin(yaw);

		if (latitude_ot != 0)
		{
			dis_ot = distance(longitude_ot, latitude_ot, gga.longitude, gga.latitude);
			dis_ot_g = distance(longitude_ot, latitude_ot, gga.longitude, gga.latitude);
		}

		arg = atan2(-torig[3] * torig[0] + torig[2] * torig[1], torig[2] * torig[0] + torig[3] * torig[1]);

		if (j == 0)
		{
			motor.setDirForward(1, 1);
			j++;
		}

		p = arg * 39.15;

		if (p >= 123)
		{
			p = 123;
		}
		else if (p <= -123)
		{
			p = -123;
		}

		if (dis < 2)
		{
			vTaskSuspend(NULL);
		}
		else if (dis_ot >= 2)
		{

			motor.forward(900 + p, 900 - p);
		}
		else if (dis - dis_ot_g <= 0)
		{

			motor.forward(900 + p, 900 - p);
		}
		else
		{

			motor.stop();
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