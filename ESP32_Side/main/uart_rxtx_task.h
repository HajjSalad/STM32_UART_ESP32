#ifndef UART_RXTX_TASK_H
#define UART_RXTX_TASK_H

/**
 * @file  uart_rxtx_task.h
 * @brief 
*/

#include <stdint.h>

// ESP32 side - high bit 1  
#define SEQ_ESP32_BASE  0x80
// seq: 0x80, 0x81, 0x82 ... 0xFF

/** @brief Start of frame delimiter. */
#define SOF_BYTE 0xAA

/** @brief End of frame delimiter. */
#define EOF_BYTE 0x55

#define RX_BUF_SIZE             50
#define MAX_PAYLOAD_SIZE        30

typedef struct {
    uint8_t seq;
    uint8_t flag_handshake_ack;
    uint8_t flag_data_ack;
} UART_RX_Context;

typedef enum {
    PKT_HANDSHAKE_REQ = 0x01,
    PKT_HANDSHAKE_ACK = 0x02,
    PKT_DATA          = 0x03,
    PKT_DATA_ACK      = 0x04,
} UART_PacketType_t;

typedef struct {
    uint8_t  sof;
    uint8_t  seq_num;
    uint8_t  type;
    uint16_t crc;
    uint8_t  eof;
} UART_ACK_Packet_t;

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
    uint8_t  type;
    uint8_t  length;
    uint8_t  payload[MAX_PAYLOAD_SIZE];
    uint16_t crc;
    uint8_t  eof;
} UART_Data_Packet_t;

// Function Prototypes
void uart_rxtx_task_init(void);

#endif      // UART_RXTX_TASK_H