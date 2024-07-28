#include "FreeRTOSConfig.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../lib/freertos/FreeRTOS-Kernel/include/task.h"
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <math.h>


int main(){

   SemaphoreHandle_t xMutax;
   xMutax=xSemaphoreCreateMutex();

   xTaskCreate(task_print,"task_print",256,NULL,1,NULL);
   xTaskCreate(task_print2,"task_print2",256,NULL,2,NULL);

   vTaskStartScheduler();
   while(1);
}

void task_print(void *print){
   printf("お好み焼き");
}

void task_print2(void *print2){
   printf("焼きそば");
}