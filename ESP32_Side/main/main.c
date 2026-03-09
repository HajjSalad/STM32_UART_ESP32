/**
 * @file  main.c
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "priorities.h"
#include "uart_driver.h"
#include "uart_tx_task.h"
#include "uart_rx_task.h"
#include "uart_router_task.h"
#include "queue_push_task.h"

static const char *TAG = "MAIN";

QueueHandle_t tx_queue;
QueueHandle_t rx_queue;

void app_main()
{
    printf("*** Program Start ***\n");

    // Initialize UART2
    ESP_ERROR_CHECK(uart2_init());         
    
    // System-wide initializations
    ESP_ERROR_CHECK(nvs_flash_init());                      // Non-volatile storage
    ESP_ERROR_CHECK(esp_netif_init());                      // LwIP TCP/IP stack
    ESP_ERROR_CHECK(esp_event_loop_create_default());       // System event loop

    tx_queue = xQueueCreate(10, sizeof(TXQueue_Item_t));
    if (tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create tx_queue\n");
        return;
    }

    rx_queue = xQueueCreate(10, sizeof(RXQueue_Item_t));
    if (rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create rx_queue\n");
        return;
    }

    // Create application tasks    
    queue_push_task_init();             // Priority: 4
    uart_tx_task_init();                // Priority: 5
    uart_rx_task_init();                // Priority: 5
    uart_router_task_init();            // Priority: 6


    printf("All tasks created\n");
}
