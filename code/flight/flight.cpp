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
#include "semphr.h"
#include "portmacro.h"
#include "projdefs.h"
#include "croutine.h"
#include "queue.h"
#include "list.h"

#include "../lib/imu/src/imu.h"
#include "../lib/ikafly_pin.h"
#include "../lib/rf/src/rf.h"
#include "../lib/motor/src/umotor.h"
#include "../lib/press/src/uprs.h"
#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../lib/log/src/littlefs.hpp"
#include "../lib/cam/src/cam.h"
#include "../lib/tjpg/src/tjpg.hpp"

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
uint32_t latitude_s = 0;
uint32_t longitude_s = 0;
uint32_t log_w[8] = {0};
uint32_t red_area[15] = {0};
int cgg = 0; // increment when gga is resdable
int cgg_o = 0;
int stacking = 0; // when ikafly is tacking, this is changed 1
int landed = 0;

nmeap_context_t nmea;
nmeap_gga_t gga;
IMU imu(I2C);
Press prs(I2C, 0x77);
Motor motor;
Radio radio(24, 22);
LFS lfs;
lfs_file_t file;
Cam cam(I2C, XCLK, Y2_PIO_BASE, PIO, PIO_SM, DMA_CH);

TJPGD tjpg;

// make semaphoreHandle
SemaphoreHandle_t xMutex;

// make taskandle
TaskHandle_t landing_h;
TaskHandle_t get_gnss_h;
TaskHandle_t get_imu_h;
TaskHandle_t stack_h;
TaskHandle_t g_motor_control_h;
TaskHandle_t send_h;
TaskHandle_t receive_h;
TaskHandle_t log_h;
TaskHandle_t cam_motor_control_h;

// melt nichrom line with heat
void nichrom()
{
	gpio_init(pin_nichrome_left);
	gpio_init(pin_nichrome_right);

	gpio_set_dir(pin_nichrome_left, 0);
	gpio_set_dir(pin_nichrome_right, 0);

	gpio_put(pin_nichrome_left, 1);
	gpio_put(pin_nichrome_right, 1);

	vTaskDelay(500);

	gpio_put(pin_nichrome_left, 0);
	gpio_put(pin_nichrome_right, 0);
}

void task_landing(void *landing)
{
	int up_s = 0;
	TickType_t lastunlock_lan;
	lastunlock_lan = xTaskGetTickCount();

	printf("press init");
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
		if (alt_cm - alt_av <= 0.1 && alt_av - alt_cm <= 0.1)
		{
			printf("ground");
			if (up_s >= 10)
			{
				nichrom();

				landed = 1;

				vTaskResume(get_imu_h);
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
		};
		alt_old = alt_cm;

		up_s++;

		if (up_s >= 10)
		{
			nichrom();
			printf("landing_delete");
			landed = 1;
			vTaskDelete(NULL);
		}

		vTaskDelayUntil(&lastunlock_lan, pdMS_TO_TICKS(100));
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

	vTaskResume(get_gnss_h);
	float euler[3];
	vTaskSuspend(NULL);

	while (1)
	{
		// if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1000)) == 1)
		//{

		imu.update();
		imu.getAttEuler(euler);

		yaw = euler[2];

		// xSemaphoreGive(xMutex);
		vTaskDelayUntil(&lastunblock_imu, pdMS_TO_TICKS(20));
		printf("imu");
		//}
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

		// if (xSemaphoreTake(xMutex, (TickType_t)0xfffffff) == 1)
		//{
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
		// xSemaphoreGive(xMutex);
		//}
		vTaskDelayUntil(&lastunblock_receive, pdMS_TO_TICKS(4000));
	}
}

void task_send(void *p)
{

	int i;
	TickType_t ltusend;
	ltusend = xTaskGetTickCount();
	vTaskSuspend(NULL);

	while (1)
	{
		// if (xSemaphoreTake(xMutex, (TickType_t)0xffffff) == 1)
		//{

		for (i = 0; i < 4; i++)
		{
			packet[i] = latitude_s >> 8 * (3 - i);
		}
		for (i = 0; i < 4; i++)
		{
			packet[i + 4] = longitude_s >> 8 * (3 - i);
		}
		// packet[8] = stacking;

		printf("sending\n");
		radio.send(packet);

		printf("j");
		// xSemaphoreGive(xMutex);
		//}

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
}

void task_get_gnss(void *get_gnss)
{
	vTaskSuspend(NULL);
	int ch;
	int j = 0;
	TickType_t lastunblock_gnss;
	lastunblock_gnss = xTaskGetTickCount();
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
		// if (xSemaphoreTake(xMutex, (TickType_t)0xfffffff) == 1)
		{
			ch = uart_getc(UART);
			nmeap_parse(&nmea, ch);
			// xSemaphoreGive(xMutex);

			if (cgg_o != cgg)
			{
				if (xSemaphoreTake(xMutex, (TickType_t)0xfffff) == 1)
				{
					if (gga.latitude != 0 && gga.longitude != 0)
					{
						if (j == 0)
						{
							printf("vv");
							j++;
							vTaskResume(landing_h);
							vTaskResume(log_h);
						}

						dis = distance(goal_longitude, goal_latitude, gga.longitude, gga.latitude);

						latitude_s = gga.latitude * 10000000;
						longitude_s = gga.longitude * 10000000;
						
					}

					if (landed == 1)
					{
						printf("ss");
						vTaskResume(g_motor_control_h);
						vTaskResume(receive_h);
						vTaskResume(send_h);
						vTaskResume(get_imu_h);
						landed++;
					}
					xSemaphoreGive(xMutex);
				}

				vTaskDelayUntil(&lastunblock_gnss, pdMS_TO_TICKS(100));
			}
		}
	}
	vTaskDelay(1);
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

		// if (xSemaphoreTake(xMutex, (TickType_t)0xfffffff) == 1)
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

			// xSemaphoreGive(xMutex);

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
			}

			/*if (j == 0)
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

			motor.forward(900 + p, 900 - p);
			if (dis <= 2.0)
			{
				motor.stop();
				vTaskSuspend(NULL);
			}
			vTaskDelayUntil(&lastunblock_gmotor, pdMS_TO_TICKS(200));*/
		}

		vTaskDelayUntil(&lastunblock_gmotor, pdMS_TO_TICKS(500));
	}
}

void task_log(void *log)
{
	vTaskSuspend(NULL);

	TickType_t lastunlocklog;
	lastunlocklog = xTaskGetTickCount();

	clock_t time_f;

	uint8_t dis_log;

	while (1)
	{
		if (xSemaphoreTake(xMutex, (TickType_t)0xfffff) == 1)
		{
			printf("log\n");
			time_f = clock() / CLOCKS_PER_SEC;
			dis_log = dis * 10;
			log_w[0] = time_f;
			log_w[1] = latitude_s;
			log_w[2] = longitude_s;
			log_w[3] = dis_log;

			if (landed = 0)
			{
				log_w[4] = alt_cm;
			}

			lfs.file_open(&file, "ikafly_log", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);

			lfs.file_write(&file, &log_w, sizeof(log_w));

			lfs.file_sync(&file);

			xSemaphoreGive(xMutex);
		}

		vTaskDelayUntil(&lastunlocklog, pdMS_TO_TICKS(2000));
		//vTaskDelay(2000);
	}
}

static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect);
void count_vert(uint8_t vert[5], uint8_t area[15]);

static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect)
{
	// ref:https://qiita.com/yosihisa/items/c326e59ca5d65f35a181
	uint8_t *src;
	uint16_t x, y; //, bws;
	uint8_t r, g, b;
	double h = 0, s, v;
	uint32_t bitshift = 0;

	//    printf("\n");
	src = (uint8_t *)bitmap;
	//    bws = N_BPP * (rect->right - rect->left + 1);
	for (y = 0; y < (rect->bottom - rect->top + 1); y++)
	{
		for (x = 0; x < (rect->right - rect->left + 1); x += 1)
		{
			//            printf("y: %d\n", y);
			r = *(src + 3 * (y * (rect->right - rect->left + 1) + x));
			g = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 1);
			b = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 2);
			//            printf("(%03d,%03d,%03d),", r,g,b);
			double MAX = max((max(r, g)), b);
			double MIN = min((min(r, g)), b);
			v = MAX / 256 * 100;

			if (MAX == MIN)
			{
				h = 0;
				s = 0;
			}
			else
			{
				if (MAX == r)
					h = 60.0 * (g - b) / (MAX - MIN) + 0;
				else if (MAX == g)
					h = 60.0 * (b - r) / (MAX - MIN) + 120.0;
				else if (MAX == b)
					h = 60.0 * (r - g) / (MAX - MIN) + 240.0;

				if (h > 360.0)
					h = h - 360.0;
				else if (h < 0)
					h = h + 360.0;
				s = (MAX - MIN) / MAX * 100.0;
			}

			if (h > 360.0)
				h -= 360;

			// 赤色の判定
			if ((h >= H_MIN_1 && h <= H_MAX_1) || (h >= H_MIN_2 && h <= H_MAX_2))
			{
				if ((s >= S_MIN && s <= S_MAX) && (v >= V_MIN && v <= V_MAX))
				{
					// printf("red!: (rx,ry)=(%d,%d)\n", rect->left+x, rect->top+y);
					bitshift = (rect->left + x);
					red_area[rect->top + y] |= (1 << bitshift);
				}
			}
		}
	}

	return 1; // Continue to decompress
}

void count_vert(uint8_t vert[5], uint32_t area[15])
{
	// count ones in vertically devided area
	// [4] [3] [2] [1] [0]
	// max count in each area: 4*15=60
	memset(vert, 0, 5);
	uint8_t bit;
	for (uint8_t y = 0; y < 15; y++)
	{
		for (int8_t x = 20 - 1; x >= 0; x--)
		{
			bit = (area[y] >> x) & 1;
			if (bit == 1)
			{
				if ((x >= 0) && (x < 4))
					vert[0]++;
				else if (x < 8)
					vert[1]++;
				else if (x < 12)
					vert[2]++;
				else if (x < 16)
					vert[3]++;
				else
					vert[4]++;
			}
		}
	}
}

void task_cam_motor_control(void *p)
{
	TickType_t lastunlock_cam;
	lastunlock_cam = xTaskGetTickCount();

	uint32_t cnt = 0;
	uint8_t vert[5] = {0};

	while (1)
	{
		printf("%d: ", cnt++);
		cam.capture();
		while (!cam.isCaptureFinished())
			tight_loop_contents();
		uint32_t size = cam.getJpegSize();
		printf("last: ");
		for (uint32_t i = size - 10; i < size; i++)
			printf("%02x", cam.image_buf[i]);
		printf(", size: %d\n", size);

		memset(red_area, 0, sizeof(red_area));
		JRESULT res = tjpg.prepare(cam.image_buf, size);
		if (res == JDR_OK)
		{
			printf("Image size is %u x %u.\n%u bytes of work area is free.\n", tjpg.jdec.width, tjpg.jdec.height, tjpg.jdec.sz_pool);
			res = tjpg.decomp(my_out_func, 3);
			// 160x120 -> (1/2^3) -> 20x15
			if (res == JDR_OK)
			{
				printf("\rDecompression succeeded.\n");
				// print red_area (20x15)
				uint8_t bit;
				for (uint8_t y = 0; y < 15; y++)
				{
					//                   std::cout << std::bitset<32>(red_area[y]) << std::endl;
					for (int8_t i = 20 - 1; i >= 0; i--)
					{
						bit = (red_area[y] >> i) & 1;
						printf("%u", bit);
					}
					printf("\n");
				}

				count_vert(vert, red_area);
				printf("count: %d %d %d %d %d\n", vert[4], vert[3], vert[2], vert[1], vert[0]);
			}
			else
			{
				printf("jd_decomp() failed (rc=%d)\n", res);
			}
		}
		else
		{
			printf("jd_prepare() failed (rc=%d)\n", res);
		}

		int8_t dire = 0;
		for (int8_t q = 0; q < 3; q++)
		{
			if (vert[q + 1] >= vert[q])
			{
				dire = q + 1;
			}
		}

		switch (dire)
		{

		case 0:

			motor.forward(800, 1023);

			break;

		case 1:

			motor.forward(900, 1023);

			break;

		case 2:

			motor.forward(1023, 1023);

			break;

		case 3:

			motor.forward(900, 1023);

			break;

		case 4:

			motor.forward(800, 1023);

			break;
		}

		vTaskDelayUntil(&lastunlock_cam, pdMS_TO_TICKS(200));
	}
}

void debug(void *p)
{
	TickType_t deb;
	deb = xTaskGetTickCount();

	while (1)
	{
		printf("okokokokokoko\n");
		vTaskDelay(1000);
		printf("jdjdjdjd\n");
		vTaskDelayUntil(&deb, pdMS_TO_TICKS(1000));
		printf("gggg");
	}
}

int main(void)
{
	stdio_init_all();
	sleep_ms(2000);

	i2c_init(I2C, 400 * 1000);

	gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);

	radio.init();

	sleep_ms(1000);

	motor.init(pin_left_begin, pin_right_begin);

	sleep_ms(1000);

	cam.init();
	cam.enableJpeg();

	printf("q");

	int st=0;

	st=lfs.init();

	printf("%d",st);

	lfs.format();

	lfs.mount();


	printf("q");

	xMutex = xSemaphoreCreateMutex();

	
	xTaskCreate(task_get_gnss, "task_get_gnss", 1024, NULL, 5, &get_gnss_h);

	xTaskCreate(task_landing, "task_landing", 1024, NULL, 4, &landing_h);

	xTaskCreate(task_g_motor_control, "task_g_motor_control", 1024, NULL, 3, &g_motor_control_h);

	// xTaskCreate(task_stack,"task_stack",256,NULL,3,&stack_h);

	xTaskCreate(task_get_imu, "task_get_imu", 1024, NULL, 1, &get_imu_h);

	xTaskCreate(task_receieve, "task_receieve", 1024, NULL, tskIDLE_PRIORITY, &receive_h);

	xTaskCreate(task_send, "task_send", 1024, NULL, 2, &send_h);

	xTaskCreate(task_log, "task_log", 1024, NULL, 6, &log_h);

	// xTaskCreate(debug,"debug",256,NULL,1,NULL);

	// xTaskCreate(task_cam_motor_control, "task_cam_motor_contol", 1024, NULL, 4, &cam_motor_control_h);

	vTaskStartScheduler();
	while (1)
	{
	};
}