/**
 * @file  queue_push_task.c
 * @brief 
*/

#include "stm32f446xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>

#include "queue_push_task.h"
#include "shared_resources.h"

static const TXQueue_Item_t data = {
    .sensorData.temperature = 2540,
    .sensorData.pressure    = 2310,
};

void vTaskQueuePush(void *pvParameters)
{
    while(1)
    {
        // Queue sensor data
        if (xQueueSend(xTXQueue, &data, 0) == pdPASS) {
            LOG("SensorData queued");
        } else {
            LOG("TX queue full, SensorData dropped");
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}