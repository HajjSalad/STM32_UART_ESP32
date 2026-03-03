#ifndef UART_H
#define UART_H

/**
 * @file  uart.h
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
#define RX_BUF_SIZE         40  // Fixed buffer size for incoming data

// ESP32 side - high bit 1  
#define SEQ_ESP32_BASE  0x80
// seq: 0x80, 0x81, 0x82 ... 0xFF

extern QueueHandle_t uart_2_queue;

typedef enum {
    PKT_DATA          = 0x01,
    PKT_HANDSHAKE_REQ = 0x02,
    PKT_HANDSHAKE_ACK = 0x03,
    PKT_DATA_ACK      = 0x04,
} UART_PacketType_t;

typedef struct {
    uint8_t  sof;
    uint8_t  seq_num;
    uint8_t  type;
    uint16_t crc;
    uint8_t  eof;
} UART_Handshake_Packet_t;

// Function Prototype
esp_err_t uart2_init(void);
void uart_rxtx_task_init(void);

#endif  // UART2_H