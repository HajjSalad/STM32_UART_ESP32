#ifndef UART_ROUTER_TASK_H_
#define UART_ROUTER_TASK_H_

/**
 * @file  uart_router_task.h
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

#include "shared_resources.h"

#define ROUTER_BUFFER_SIZE     64

typedef struct {
    uint8_t buffer[ROUTER_BUFFER_SIZE];
    uint8_t buffer_index;
    uint8_t len;
} UART_Router_Context;

// Function Prototypes
void uart_router_task_init(void);

#endif          // UART_ROUTER_TASK_H_