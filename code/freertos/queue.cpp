#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "../lib/freertos/config/FreeRTOSConfig.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/queue.h"


int32_t atai;
int32_t kotae;

uint16_t i = 0;

QueueHandle_t xqueue;

void taskA(void *a)
{
    xqueue = xQueueCreate(1, sizeof(atai));

    while (1)
    {
        atai = i;
        i++;
        xQueueSendToFront(xqueue, &atai, (TickType_t)0);
        vTaskDelay(100);
    }
}

void taskB(void *b)
{
    while (1)
    {
        xQueueReceive(xqueue, &kotae, (TickType_t)5);

        printf("%d", kotae);
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    xTaskCreate(taskA, "taskA", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1)
    {
    };
}
