#ifndef UART_RX_H_
#define UART_RX_H_

/**
 * @file  uart_rx.h
 * @brief 
*/

#include <stdio.h>
#include <stdint.h>

#define RX_BUFFER_SIZE      10

typedef struct {
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t buffer_index;
} UART_RX_Context;


// Function Prototypes
void uart_rx_push_byte(uint8_t byte);

#endif