#ifndef UART_DRIVER_H
#define UART_DRIVER_H

/**
 * @file  uart_driver.h
 * @brief Public API for UART2 peripheral.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"

#define UART_2_TX           17
#define UART_2_RX           16
#define UART_NUM2           UART_NUM_2
#define BUF_SIZE            1024

extern QueueHandle_t uart_2_queue;

// Function Prototype
esp_err_t uart2_init(void);

#endif  // UART_DRIVER_H