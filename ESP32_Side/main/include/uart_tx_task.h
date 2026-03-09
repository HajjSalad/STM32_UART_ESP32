#ifndef UART_TX_TASK_H_
#define UART_TX_TASK_H_

/**
 * @file  uart_tx_task.h
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include "shared_resources.h"

// ESP32 side - high bit 1
#define SEQ_ESP32_BASE  0x80                // seq: 0x80, 0x81, 0x82 ... 0xFF

typedef enum {
    TX_STATE_SEND_HANDSHAKE,
    TX_STATE_WAIT_HANDSHAKE_ACK,
    TX_STATE_SEND_COMMAND,
    TX_STATE_WAIT_COMMAND_ACK,
    TX_STATE_IDLE
} UART_TX_State_t;

typedef struct {
    UART_TX_State_t state;
    uint8_t         seq;
    uint8_t         retry_count;
    uint8_t         timestamp;
    uint8_t         flag_handshake_ack;
    uint8_t         flag_command_ack;
} UART_TX_Context;

// Function Prototypes
void uart_tx_task_init(void);
void uart_tx_set_flag(UART_PacketType_t type);

#endif      // UART_TX_TASK_H_