/**
 * @file  uart_tx_task.c
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>

#include "crc_16.h"
#include "priorities.h"
#include "uart_driver.h"
#include "uart_tx_task.h"
#include "shared_resources.h"

static const char *TAG = "TX";

// Static Function Prototypes
static void send_handshake_request(uint8_t seq);
static void send_command_packet(uint8_t seq, const TXQueue_Item_t *pQueueItem);

// Instantiate a TX context
static UART_TX_Context tx_ctx = {
    .state       = TX_STATE_SEND_HANDSHAKE,
    .seq         = SEQ_ESP32_BASE,
    .retry_count = 0,
};

static void uart_tx_task(void *pvParameters)
{
    TXQueue_Item_t command = {0};

    while(1)
    {
        if (xQueueReceive(tx_queue, &command, portMAX_DELAY)) 
        {
            // Reset for new transaction
            tx_ctx.state       = TX_STATE_SEND_HANDSHAKE;
            tx_ctx.retry_count = 0;

            // Run state machine to completion for each item in the queue
            while (tx_ctx.state != TX_STATE_IDLE) 
            {
                switch(tx_ctx.state) 
                {
                    case TX_STATE_SEND_HANDSHAKE:
                        printf("\n--- Starting new TX cycle ---\n");
                        send_handshake_request(tx_ctx.seq);             // Request handshake
                        tx_ctx.timestamp = xTaskGetTickCount();         // Get timestamp
                        tx_ctx.retry_count++;                           // Increment retry count
                        tx_ctx.state = TX_STATE_WAIT_HANDSHAKE_ACK;     // Move to wait for ACK
                        break;
                    case TX_STATE_WAIT_HANDSHAKE_ACK:
                        if (tx_ctx.flag_handshake_ack) {                // Handshake ACK flag received
                            tx_ctx.flag_handshake_ack = 0;              // Clear the flag
                            tx_ctx.retry_count = 0;                     // Reset the retry count
                            tx_ctx.state = TX_STATE_SEND_COMMAND;       // Move to send COMMAND
                        }
                        else if ((xTaskGetTickCount() - tx_ctx.timestamp) >= pdMS_TO_TICKS(UART_TIMEOUT_MS))
                        {
                            if (tx_ctx.retry_count < RETRY_MAX) {               // Max retry not reached
                                tx_ctx.state = TX_STATE_SEND_HANDSHAKE;         // Send handshake again
                                printf("[0x%02X] HANDSHAKE_ACK timeout. Retrying...\n", tx_ctx.seq);
                            } else {                                            // Max retry reached
                                tx_ctx.retry_count = 0;                         // Reset the retry count
                                tx_ctx.state = TX_STATE_IDLE;                   // Move to idle state
                                printf("[0x%02X] HANDSHAKE failed after %d attempts. Going IDLE\n", tx_ctx.seq, RETRY_MAX);
                            }
                        }
                        break;
                    case TX_STATE_SEND_COMMAND:
                        send_command_packet(tx_ctx.seq, &command);          // Send command
                        tx_ctx.retry_count++;                               // Increment retry count
                        tx_ctx.timestamp = xTaskGetTickCount();             // Get timestamp
                        tx_ctx.state = TX_STATE_WAIT_COMMAND_ACK;           // Move to command ACK
                        break;
                    case TX_STATE_WAIT_COMMAND_ACK:
                        if (tx_ctx.flag_command_ack) {                      // Command ACK flag received
                            tx_ctx.flag_command_ack = 0;                       // Reset ACK flag
                            tx_ctx.state = TX_STATE_IDLE;                   // Move to IDLE
                            printf("--- Cycle TX complete -------\n");
                        } else if ((xTaskGetTickCount() - tx_ctx.timestamp) >= pdMS_TO_TICKS(UART_TIMEOUT_MS)) 
                        {
                            if (tx_ctx.retry_count < RETRY_MAX) {           // Retry less than MAX
                                tx_ctx.state = TX_STATE_SEND_COMMAND;       // Send command again
                                printf("[0x%02X]COMMAND_ACK timeout. Retrying...\n", tx_ctx.seq);
                            } else {
                                tx_ctx.retry_count = 0;                     // Reset retry count
                                tx_ctx.state = TX_STATE_IDLE;               // Move to IDLE
                                printf("[0x%02X]Command send failed after %d attempts. Going IDLE\n", tx_ctx.seq, RETRY_MAX);
                            }
                        }
                        break;
                    case TX_STATE_IDLE:
                        tx_ctx.seq = (tx_ctx.seq + 1) & 0x7F;               // Increment seq number
                        break;
                }
            }
        }
    }
}

void uart_tx_set_flag(UART_PacketType_t type)
{
    switch (type)
    {
        case PKT_HANDSHAKE_ACK: tx_ctx.flag_handshake_ack = 1; break;
        case PKT_COMMAND_ACK:   tx_ctx.flag_command_ack   = 1; break;
        default: break;
    }
}

static void send_handshake_request(uint8_t seq)
{
    UART_Handshake_Packet_t pkt;

    pkt.sof       = SOF_BYTE;
    pkt.version   = PROTOCOL_VERSION;
    pkt.device_id = DEVICE_ID_ESP32;
    pkt.seq_num   = seq;
    pkt.type      = PKT_HANDSHAKE_REQ;
    pkt.crc       = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);   // Exclude 3 last bytes (crc & eof)
    pkt.eof       = EOF_BYTE;

    printf("[0x%02X] TX HANDSHAKE_REQ  : SOF:0x%02X VER:0x%02X DEV:0x%02X SEQ:0x%02X TYPE:0x%02X CRC:0x%02X 0x%02X EOF:0x%02X\n",
        tx_ctx.seq, pkt.sof, pkt.version, pkt.device_id, pkt.seq_num, pkt.type, pkt.crc >> 8, pkt.crc & 0x00FF, pkt.eof);
    
    uart_write_bytes(UART_NUM2, (uint8_t*)&pkt, sizeof(pkt));
}

static void send_command_packet(uint8_t seq, const TXQueue_Item_t *pQueueItem)
{
    UART_Command_Packet_t pkt;

    pkt.sof       = SOF_BYTE;
    pkt.version   = PROTOCOL_VERSION;
    pkt.device_id = DEVICE_ID_STM32;
    pkt.seq_num   = seq;
    pkt.type      = pQueueItem->type;
    pkt.code      = pQueueItem->code;
    pkt.crc       = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);   // Exclude 3 last bytes (crc & eof)
    pkt.eof       = EOF_BYTE;

    printf("[0x%02X] TX Command : SOF:0x%02X VER:0x%02X DEV:0x%02X SEQ:0x%02X TYPE:0x%02X CODE:0x%02X CRC:0x%02X 0x%02X EOF:0x%02X\n",
        tx_ctx.seq, pkt.sof, pkt.version, pkt.device_id, pkt.seq_num, pkt.type, pQueueItem->code, pkt.crc >> 8, pkt.crc & 0x00FF, pkt.eof);
    
    uart_write_bytes(UART_NUM2, (uint8_t*)&pkt, sizeof(pkt));
}


void uart_tx_task_init(void)
{
    xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, TASK_PRIO_5, NULL);
}