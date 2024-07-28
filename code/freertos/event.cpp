#include "../lib/freertos/config/FreeRTOSConfig.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include "../lib/freertos/FreeRTOS-Kernel/include/event_groups.h"
#include <stdio.h>
#include <stdint.h>

EventGroupHandle_t event_a;

const uint32_t handle_a(1 << 0);
const uint32_t handle_b(1 << 1);
const uint32_t handle_c(1 << 2);
const uint32_t handle_d(1 << 3);

void taskA(void )
{
    while (1)
    {
        printf("taskA");
    }
}

int main(void)
{

    event_a = xEventGroupCreate();

    xTaskCreate(taskA, "taskA", 1, NULL, 1024, NULL);
    xTaskCreate(taskB, "taskB", 1, NULL, 1024, NULL);
    xTaskCreate(taskC, "taskC", 1, NULL, 1024, NULL);
    xTaskCreate(taskD, "taskD", 1, NULL, 1024, NULL);
}