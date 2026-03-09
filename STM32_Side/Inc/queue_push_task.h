#ifndef QUEUE_PUSH_TASK_H_
#define QUEUE_PUSH_TASK_H_

/**
 * @file  queue_push_task.h
 * @brief 
*/

#include "stm32f446xx.h"
#include "FreeRTOS.h"
#include "queue.h"

// Function Prototypes
void vTaskQueuePush(void *pvParameters);

#endif      // QUEUE_PUSH_TASK_H_