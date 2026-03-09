#ifndef UART_RX_H_
#define UART_RX_H_

/**
 * @file  uart_rx_task.h
 * @brief 
*/

#include <stdio.h>
#include <stdint.h>
#include "shared_resources.h"

// typedef enum {
//     STATE_RECEIVE_HANDSHAKE,
//     STATE_WAIT_DATA,
//     STATE_RECEIVE_DATA, 
//     STATE_IDLE
// } UART_RX_State_t;

// typedef struct {
//     UART_RX_State_t state;
//     uint8_t         seq;
//     uint8_t         retry_count;
//     uint8_t         timestamp;
//     uint8_t         flag_handshake_req;
//     uint8_t         flag_data;
// } UART_RX_Context;

// Function Prototypes
void vTaskRX(void *pvParameters);

#endif