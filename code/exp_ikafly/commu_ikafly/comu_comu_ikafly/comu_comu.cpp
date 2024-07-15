#include <stdio.h>
#include <stdint.h>
#include "pico/stdio.h"
#include "hardware/timer.h"
#include "../../../lib/rf/src/rf.h"
#include "../../../lib/ikafly_pin.h"
#include "FreeRTOSConfig.h"
#include "../../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "semphr.h"

Radio radio(24, 22);
uint8_t packet[32];
uint8_t packet_r[32] = {0};
int k;

TickType_t lastre;
TickType_t lastse;
SemaphoreHandle_t xMutex;

TaskHandle_t tsend;
TaskHandle_t treceive;

void task_receive(void *p)
{
    lastre = xTaskGetTickCount();
    while (1)
    {

        // if (xSemaphoreTake(xMutex, (TickType_t)pdMS_TO_TICKS(1000)) == 1){

        radio.receive(packet_r);

        vTaskPrioritySet(tsend, 1);
        vTaskPrioritySet(treceive, 2);

        for (int k = 0; k < 5; k++)
        {
            radio.receive(packet_r);
        }

        for (int i = 0; i < 32; i++)
        {
            printf("%d", packet_r[i]);
        }

        vTaskPrioritySet(treceive, 1);
        vTaskPrioritySet(tsend, 2);

      //  xSemaphoreGive(xMutex);

         
        vTaskDelay(1000);
         }
   // }
}

void task_send(void *p)
{
    while (1)
    {
       // if (xSemaphoreTake(xMutex, (TickType_t)pdMS_TO_TICKS(1000)) == 1)
        //{

            radio.send(packet);

            printf("send");

            xSemaphoreGive(xMutex);

            vTaskDelay(2000);
       // }
       
}
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    printf("%s\n", __FILE_NAME__);

    radio.init();
    sleep_ms(2000);

    for (int i = 0; i < 32; i++)
    {
        packet[i] = 2;
    }

    xMutex = xSemaphoreCreateMutex();

    xTaskCreate(task_receive, "task_receive", 516, NULL, 1, &tsend);
    xTaskCreate(task_send, "task_send", 516, NULL, 2, &treceive);
    vTaskStartScheduler();
    while (1)
    {
    };
}