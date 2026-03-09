#ifndef UART_RX_TASK_H_
#define UART_RX_TASK_H_

/**
 * @file  uart_tx_task.h
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdint.h>

#include "shared_resources.h"

// Function Prototypes
void uart_rx_task_init(void);

#endif      // UART_RX_TASK_H_