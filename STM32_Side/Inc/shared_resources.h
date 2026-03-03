#ifndef SHARED_RESOURCES_H_
#define SHARED_RESOURCES_H_

/**
 * @file  shared_resources.h
 * @brief Shared constants, macros, and type definitions used across all UART transport layer modules.
*/

/** @brief Format for printf */
#define LOG(fmt, ...)  printf( (fmt "\n\r"), ##__VA_ARGS__)

/** @brief Start of frame delimiter. */
#define SOF_BYTE 0xAA

/** @brief End of frame delimiter. */
#define EOF_BYTE 0x55

/**
 * @brief UART packet type identifiers.
*/
typedef enum {
    PKT_HANDSHAKE_REQ = 0x01,
    PKT_HANDSHAKE_ACK = 0x02,
    PKT_DATA          = 0x03,
    PKT_DATA_ACK      = 0x04,
} UART_PacketType_t;

#endif      // SHARED_RESOURCES_H_