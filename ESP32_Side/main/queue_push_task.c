/**
 * @file  queue_push_task.c
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include <stdint.h>

#include "priorities.h"
#include "uart_tx_task.h"
#include "queue_push_task.h"
#include "shared_resources.h"

static const TXQueue_Item_t command = {
    .type    = PKT_COMMAND,
    .code    = CMD_SET_THRESHOLD,
};

static void queue_push_task(void *pvParameters)
{
    while(1)
    {
        // Queue the command
        if (xQueueSend(tx_queue, &command, 0) == pdPASS) {
            printf("Command queued\n");
        } else {
            printf("tx_queue full, Command dropped\n");
        }
        vTaskDelay(QUEUE_PUSH_TASK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void queue_push_task_init(void)
{
    xTaskCreate(queue_push_task, "queue_push_task", 2048, NULL, TASK_PRIO_4, NULL);
}