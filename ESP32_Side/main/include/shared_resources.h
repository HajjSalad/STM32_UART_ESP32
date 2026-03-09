#ifndef SHARED_RESOURCES_H_
#define SHARED_RESOURCES_H_

/**
 * @file  shared_resources.h
 * @brief Shared constants, macros, and type definitions used across all UART transport layer modules.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

/* ---  These are agreed on by both sides  --- */
#define SOF_BYTE             0xAA        // Start of frame delimiter
#define EOF_BYTE             0x55        // End of frame delimiter

#define PROTOCOL_VERSION     0x01        // Identidy protocol version
#define DEVICE_ID_STM32      0x01        // STM32 sensor node
#define DEVICE_ID_ESP32      0x02        // ESP32 cloud gateway

#define RETRY_MAX            5           // 5 tries for retransmission
#define UART_TIMEOUT_MS      1000        // Timeout in milliseconds

#define SOF_POS              0           // Position of sof in all structs
#define SEQ_POS              3           // Position of seq_num in all structs
#define TYPE_POS             4           // Position of type in all structs

#define MAX_PAYLOAD_SIZE     64          // Maximum size of payload sent by STM32
/* ---  These are agreed on by both sides  --- */

extern QueueHandle_t tx_queue;
extern QueueHandle_t rx_queue;

/* --- From ESP32 to STM32 --- */

// Commands sent to STM32 Sensor Node from ESP32 Gateway Node
typedef enum {
    CMD_START_OTA       = 0x01,
    CMD_SET_THRESHOLD   = 0x02,
} CommandCode_t;

// Item on the TXQueue
typedef struct __attribute__((packed)) {
    uint8_t         type;
    CommandCode_t   code;
} TXQueue_Item_t;

// TX sends to STM32 Sensor Node
typedef struct {
    uint8_t  sof;
    uint8_t  version;
    uint8_t  device_id;
    uint8_t  seq_num;
    uint8_t  type;
    uint16_t crc;
    uint8_t  eof;
} UART_Handshake_Packet_t;

// TX sends to STM32 Sensor Node
typedef struct {
    uint8_t  sof;
    uint8_t  version;
    uint8_t  device_id;
    uint8_t  seq_num;
    uint8_t  type;
    uint8_t  code;
    uint16_t crc;
    uint8_t  eof;
} UART_Command_Packet_t;

/**
 * @brief UART packet type identifiers.
*/
typedef enum {
    PKT_HANDSHAKE_REQ = 0x01,
    PKT_HANDSHAKE_ACK = 0x02,
    PKT_DATA          = 0x03,
    PKT_DATA_ACK      = 0x04,
    PKT_COMMAND       = 0x05,
    PKT_COMMAND_ACK   = 0x05,
} UART_PacketType_t;

typedef struct __attribute__((packed)) {
    uint16_t    temperature;
    uint16_t    pressure;
} SensorData_t;

typedef struct {
    uint8_t     seq;
    uint8_t     type;
    uint8_t     payload[MAX_PAYLOAD_SIZE];
} RXQueue_Item_t;

typedef struct {
    uint8_t  sof;
    uint8_t  seq_num;
    uint8_t  type;
    uint16_t crc;
    uint8_t  eof;
} UART_ACK_Packet_t;




// Future addition - Receive both messages & data from STM32
typedef enum {
    MSG_HEARTBEAT               = 0x01,   // STM32 alive and running
    MSG_FIRMWARE_UPDATE_OK      = 0x02,   // OTA flash succeeded
    MSG_FIRMWARE_UPDATE_FAIL    = 0x03,   // OTA flash failed
    MSG_SYSTEM_BOOT             = 0x04,   // STM32 booted/rebooted
} MessageCode_t;

typedef struct __attribute__((packed)) {
    uint8_t code;
} MessageData_t;

typedef enum {
    RX_PAYLOAD_SENSOR_DATA    = 0x01,
    RX_PAYLOAD_MESSAGE_DATA   = 0x02,
} RX_PayloadType_t;












#endif      // SHARED_RESOURCES_H_