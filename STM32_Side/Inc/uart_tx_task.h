#ifndef UART_TX_TASK_H_
#define UART_TX_TASK_H_

/**
 * @file  uart_tx_task.h
 * @brief 
*/

#include <stdio.h>
#include <stdint.h>
#include "shared_resources.h"

// STM32 side - high bit 0
#define SEQ_STM32_BASE  0x00                // seq: 0x00, 0x01, 0x02 ... 0x7F

typedef enum {
    STATE_SEND_HANDSHAKE,
    STATE_WAIT_HANDSHAKE_ACK,
    STATE_SEND_DATA,
    STATE_WAIT_DATA_ACK,
    STATE_IDLE
} UART_TX_State_t;

typedef struct {
    UART_TX_State_t state;
    uint8_t         seq;
    uint8_t         retry_count;
    uint8_t         tx_timestamp;
    uint8_t         flag_handshake_ack;
    uint8_t         flag_data_ack;
} UART_TX_Context;

// Function Prototypes
void uart_tx_set_flag(UART_PacketType_t type);
void vTaskTX(void *pvParameters);

#endif      // UART_TX_TASK_H_