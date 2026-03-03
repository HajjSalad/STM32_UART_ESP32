#ifndef UART_TX_H_
#define UART_TX_H_

/**
 * @file  uart_tx.h
 * @brief 
*/

#include <stdio.h>
#include <stdint.h>

// STM32 side - high bit 0
#define SEQ_STM32_BASE  0x00
// seq: 0x00, 0x01, 0x02 ... 0x7F

#define RETRY_MAX               5
#define UART_TIMEOUT_MS         1000U
#define MAX_PACKET_SIZE         256
#define MAX_PAYLOAD_SIZE        18

typedef enum {
    STATE_SEND_HANDSHAKE,
    STATE_WAIT_HANDSHAKE_ACK,
    STATE_SEND_DATA,
    STATE_WAIT_DATA_ACK,
    STATE_IDLE
} UART_State_t;

typedef struct {
    uint8_t  sof;
    uint8_t  seq_num;
    uint8_t  type;
    uint16_t crc;
    uint8_t  eof;
} UART_Handshake_Packet_t;

typedef struct {
    uint8_t  sof;
    uint8_t  seq_num;
    uint8_t  length;
    uint8_t  type;
    uint8_t  payload[MAX_PAYLOAD_SIZE];
    uint16_t crc;
    uint8_t  eof;
} UART_Data_Packet_t;

typedef struct {
    UART_State_t state;
    uint8_t      seq;
    uint8_t      retry_count;
    uint8_t      tx_timestamp;
    uint8_t      flag_handshake_ack;
    uint8_t      flag_data_ack;
} UART_TX_Context;

// Function Prototypes
void uart_tx_set_flag(uint8_t flag_type);


#endif      // UART_TX_H_