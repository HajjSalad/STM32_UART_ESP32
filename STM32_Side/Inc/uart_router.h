#ifndef UART_ROUTER_H_
#define UART_ROUTER_H_

/**
 * @file  uart_router.h
 * @brief 
*/

#include <stdio.h>
#include <stdint.h>
#include "shared_resources.h"

#define ROUTER_BUFFER_SIZE     sizeof(RXQueue_Item_t)

typedef struct {
    uint8_t buffer[ROUTER_BUFFER_SIZE];
    uint8_t buffer_index;
} UART_Router_Context;

// Function Prototypes
void uart_isr_push_byte(uint8_t byte);

#endif      // UART_ROUTER_H_