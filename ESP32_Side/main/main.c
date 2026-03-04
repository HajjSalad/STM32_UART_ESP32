
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "string.h"

#include "uart_driver.h"
#include "uart_rxtx_task.h"

void app_main()
{
    printf("UART Communication Initialized\n");
    printf("Waiting for messages from STM32...\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uart2_init();            // Initialize UART
    uart_rxtx_task_init();  // Create and start UART task
}
