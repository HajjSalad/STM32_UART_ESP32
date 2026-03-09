/**
 * @file  main.c
 * @brief Program entry point 
*/

#include "stm32f446xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <stdint.h>

#include "systick.h"
#include "uart_tx_task.h"
#include "uart_rx_task.h"
#include "uart_driver.h"
#include "queue_push_task.h"
#include "shared_resources.h"

#define STACK_SIZE_WORDS       (1024U)

// Global Resource handles
QueueHandle_t        xTXQueue          = NULL;
QueueHandle_t        xRXQueue          = NULL;
SemaphoreHandle_t    xTXMutex          = NULL;

/**
 * @brief FreeRTOS stack overflow hook.
 * 
 * Called automatically when stack overflow is detected. 
 * Logs the offending task name and halts the system.
*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;                // Suppress unused parameter warning
    uart2_write('!');           // Indicate stack overflow error
    while(1) {}
}

/**
 * @brief Application entry point.
 * 
 * Initializes UART peripherals, creates FreeRTOS synchronization
 * primitives and tasks, then starts the scheduler.
*/
int main(void) 
{
    BaseType_t xRet = pdFALSE;

    uart1_init();               // Initialize UART1
    uart2_init();               // Initialize UART2
    systick_init();             // Initialize SysTick

    LOG("*** Program start ***");

    // Create synchronization primitives
    xTXQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(SensorData_t));
    configASSERT(xTXQueue != NULL);

    xTXMutex = xSemaphoreCreateMutex();
    configASSERT(xTXMutex != NULL);

    xRXQueue = xQueueCreate(RX_QUEUE_LENGTH, RX_MSG_MAX_LEN);
    configASSERT(xTXQueue != NULL);

    // Create tasks
    xRet = xTaskCreate(vTaskQueuePush, "QueuePush", STACK_SIZE_WORDS, NULL, 5, NULL);
    configASSERT(xRet == pdPASS);
    xRet = xTaskCreate(vTaskTX,         "TX",       STACK_SIZE_WORDS, NULL, 4, NULL);
    configASSERT(xRet == pdPASS);
    xRet = xTaskCreate(vTaskRX,         "RX",       STACK_SIZE_WORDS, NULL, 3, NULL);
    configASSERT(xRet == pdPASS);

    LOG("Tasks created. Free heap: %u bytes", xPortGetFreeHeapSize());
    LOG("Starting scheduler...");

    vTaskStartScheduler();  

    // Should never reach here - halt if scheduler exits
    LOG("Scheduler exited unexpectedly!");
    while (1) {}
}
