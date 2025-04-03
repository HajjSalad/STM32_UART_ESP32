
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "string.h"

#define UART_0_TX 1
#define UART_0_RX 3
#define UART_2_TX 17
#define UART_2_RX 16
#define UART_NUM0 UART_NUM_0
#define UART_NUM2 UART_NUM_2
#define BUF_SIZE 1024
#define RX_BUF_SIZE 40  // Fixed buffer size for incoming data

static QueueHandle_t uart_queue;
static QueueHandle_t uart_2_queue;

void uart_init()
{
    const uart_config_t uart_config = {             
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    
    uart_param_config(UART_NUM0, &uart_config);
    uart_param_config(UART_NUM2, &uart_config);
    
    uart_set_pin(UART_NUM0, UART_0_TX, UART_0_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM2, UART_2_TX, UART_2_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM0, BUF_SIZE, BUF_SIZE, 10, &uart_queue, 0);
    uart_driver_install(UART_NUM2, BUF_SIZE, BUF_SIZE, 10, &uart_2_queue, 0);
}

static void rx_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t rx_data[RX_BUF_SIZE] = {0};

    while (1) {
        if (xQueueReceive(uart_2_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                // Clear buffer before reading
                memset(rx_data, 0, sizeof(rx_data));
                
                // Read data (limit to buffer size - 1 to keep null-terminated)
                int to_read = (event.size < sizeof(rx_data)-1) ? event.size : sizeof(rx_data)-1;
                uart_read_bytes(UART_NUM2, rx_data, to_read, portMAX_DELAY);
                
                // Print received data
                printf("Received: %s\n", rx_data);

                // Handle protocol commands
                if (strncmp((char*)rx_data, "READY?", 6) == 0) {
                    // Send "OK" when STM32 sends "READY?"
                    const char *response_ok = "OK\n";
                    uart_write_bytes(UART_NUM2, response_ok, strlen(response_ok));
                    printf("Responded with: OK\n");
                } 
                else if (strncmp((char*)rx_data, "Message from STM32", 18) == 0) {
                    // Send "ACK" when STM32 sends message
                    const char *ack_response = "ACK\n";
                    uart_write_bytes(UART_NUM2, ack_response, strlen(ack_response));
                    printf("Responded with: ACK\n\n");
                }
                else {
                    printf("Unknown data received.\n\n");
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void app_main()
{
    printf("UART Communication Initialized\n");
    printf("Waiting for messages from STM32...\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uart_init();
    xTaskCreate(rx_task, "uart_rx_task", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}
