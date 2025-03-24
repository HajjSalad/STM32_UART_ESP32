// Receive a string via UART

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"

#define UART_0_TX 1
#define UART_0_RX 3
#define UART_2_TX 17
#define UART_2_RX 16
#define UART_NUM UART_NUM_0

void init_RS232()
{
    // USB Connection
    const uart_port_t uart_num = UART_NUM;
    const int uart_buffer_size = 1024;
    QueueHandle_t uart_queue;
    QueueHandle_t uart_2_queue;

    // 1 - Setting Communication Parameters
    const uart_config_t uart_config = {             
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(uart_num, &uart_config);
    uart_param_config(UART_NUM_2, &uart_config);
    // 2 - Setting Communication Pins
    uart_set_pin(uart_num, UART_0_TX, UART_0_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_2, UART_2_TX, UART_2_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 3 - Driver Installation
    uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);
    uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_2_queue, 0);
}

static void rx_task()
{
    uint8_t rx_data[40];                     // Buffer for incoming data
    const char *ready_msg = "READY";
    const char *ack_msg = "ACK";

    while (1) {
        // Send "READY" to STM32
        uart_write_bytes(UART_NUM_2, ready_msg, strlen(ready_msg));
        printf("Sent: READY\n");

        // Small delay before reading to prevent conflicts
        vTaskDelay(pdMS_TO_TICKS(100));

        // Wait for STM32 response
        int length = uart_read_bytes(UART_NUM_2, rx_data, sizeof(rx_data) - 1, 1000 / portTICK_PERIOD_MS);
        if (length > 0) {
            rx_data[length] = '\0';  // Null-terminate
            printf("Received: %s\n", rx_data);

            // Send "ACK" to STM32
            uart_write_bytes(UART_NUM_2, ack_msg, strlen(ack_msg));
            printf("Sent: ACK\n");
        } else {
            printf("Timeout: No response from STM32\n");
        }

        // Send ready
        // const char *ready = "READY\n\r";
        // uart_write_bytes(UART_NUM_2, ready, strlen(ready));

        // // Receive data
        // int length = uart_read_bytes(UART_NUM_2, rx_data, sizeof(rx_data) - 1, 100 / portTICK_PERIOD_MS);
        // if (length > 0) {
        //     rx_data[length] = '\0';  // Null-terminate the string
        //     printf("Received: %s\n", rx_data);
        // }

        // Send Received
        // const char *receipt = "RECEIPT\n\r";
        // uart_write_bytes(UART_NUM_2, receipt, strlen(receipt));

        // //int length = uart_read_bytes(uart_num, rx_data, sizeof(rx_data) - 1, 100 / portTICK_PERIOD_MS);
        // int length = uart_read_bytes(UART_NUM_2, rx_data, sizeof(rx_data) - 1, 100 / portTICK_PERIOD_MS);
        // if (length > 0) {
        //     rx_data[length] = '\0';  // Null-terminate the string
        //     printf("Received: %s\n", rx_data);

        //     // Send acknowledgment "OK"
        //     const char *ack = "OK";
        //     uart_write_bytes(UART_NUM_2, ack, strlen(ack));
        // }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Small delay to prevent CPU overload
    }

    // const uart_port_t uart_num = UART_NUM;
    // int length = 0;
    // uint8_t rx_data[128]="\0";

    // while (1)
    // {
    //     uart_get_buffered_data_len(uart_num, (size_t *)&length); // Read data string length
    //     uart_read_bytes(uart_num, rx_data, length, 100); // Read data string from the buffer
    //     if (length>1) printf("rx_data - %.*s", length, rx_data);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
}

void app_main()
{
    printf("Receive data:\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    init_RS232();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}