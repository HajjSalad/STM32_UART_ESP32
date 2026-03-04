/**
 * @file uart2_driver.c
 * @brief
 * 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "string.h"
#include "stdio.h"

#include "uart_driver.h"
#include "uart_rxtx_task.h"

QueueHandle_t uart_2_queue;

esp_err_t uart2_init(void)
{
    const uart_config_t uart_config = {             
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t ret;
    
    // Configure parameters
    ret = uart_param_config(UART_NUM2, &uart_config);
    if (ret != ESP_OK) return ret;

    // Assign TX, RX pins
    ret = uart_set_pin(UART_NUM2, UART_2_TX, UART_2_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    // Install UART driver
    ret = uart_driver_install(UART_NUM2, BUF_SIZE, BUF_SIZE, 10, &uart_2_queue, 0);

    // Enable pattern detection interrupt — fires a UART_PATTERN_DET event
    // every time EOF_BYTE (0x55) is received, signaling end of a packet
    uart_enable_pattern_det_baud_intr(UART_NUM2, EOF_BYTE, 1, 9, 0, 0);

    // Allocate internal queue to store positions of detected EOF pattern bytes
    // Holds up to 20 pattern positions before oldest is discarded
    uart_pattern_queue_reset(UART_NUM2, 20);

    // Flush any garbage bytes that may have arrived during initialization
    uart_flush(UART_NUM2);
    
    return ret;
}

