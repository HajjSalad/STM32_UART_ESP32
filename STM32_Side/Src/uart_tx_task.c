/**
 * @file  uart_tx_task.c
 * @brief 
*/

#include "stm32f446xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <stdint.h>

#include "crc_16.h"
#include "systick.h"
#include "uart_driver.h"
#include "uart_tx_task.h"
#include "shared_resources.h"

// Static function prototypes
static void send_handshake_request(uint8_t seq);
static void send_data_packet(uint8_t seq, const TXQueue_Item_t *pQueueItem);

// Instantiate a TX context
static UART_TX_Context tx_ctx = {
    .state       = STATE_SEND_HANDSHAKE,
    .seq         = SEQ_STM32_BASE,
    .retry_count = 0,
};

void vTaskTX(void *pvParameters)
{
    (void)pvParameters;                 // Suppress unused parameter warning

    BaseType_t    xRet          = pdFALSE;
    TXQueue_Item_t queueItem    = {0U};

    while(1) 
    {
        // Block here until data is available to send
        xRet = xQueueReceive(xTXQueue, &queueItem, portMAX_DELAY);
        if (xRet != pdTRUE) {
            LOG("TX queue receive failed unexpectedly");
            continue;
        }

        // Reset for new transaction
        tx_ctx.state       = STATE_SEND_HANDSHAKE;
        tx_ctx.retry_count = 0;

        // Run state machine to completion for each item in the queue
        while (tx_ctx.state != STATE_IDLE) 
        {
            switch(tx_ctx.state) 
            {
                case STATE_SEND_HANDSHAKE:
                    LOG("\n--- Starting new TX cycle ---");
                    send_handshake_request(tx_ctx.seq);             // Request handshake
                    tx_ctx.tx_timestamp = systickGetMillis();       // Get timestamp
                    tx_ctx.retry_count++;                           // Increment retry count
                    tx_ctx.state = STATE_WAIT_HANDSHAKE_ACK;        // Move to wait for ACK
                    break;
                case STATE_WAIT_HANDSHAKE_ACK:
                    if (tx_ctx.flag_handshake_ack) {
                        tx_ctx.flag_handshake_ack = 0;
                        tx_ctx.retry_count = 0;
                        tx_ctx.state = STATE_SEND_DATA;
                    }
                    else if ((systickGetMillis() - tx_ctx.tx_timestamp) >= UART_TIMEOUT_MS)  
                    {
                        if (tx_ctx.retry_count < RETRY_MAX) {
                            tx_ctx.state = STATE_SEND_HANDSHAKE;
                            LOG("[0x%02X]HANDSHAKE_ACK timeout. Retrying...", tx_ctx.seq);
                        } else {
                            tx_ctx.retry_count = 0;
                            tx_ctx.state = STATE_IDLE;
                            LOG("[0x%02X]HANDSHAKE failed after %d attempts. Going IDLE", tx_ctx.seq, RETRY_MAX);
                        }
                    }
                    break;
                case STATE_SEND_DATA:
                    send_data_packet(tx_ctx.seq, &queueItem);
                    tx_ctx.retry_count++;
                    tx_ctx.tx_timestamp = systickGetMillis();
                    tx_ctx.state = STATE_WAIT_DATA_ACK;
                    break;
                case STATE_WAIT_DATA_ACK:
                    if (tx_ctx.flag_data_ack) {
                        tx_ctx.flag_data_ack = 0;
                        tx_ctx.state = STATE_IDLE;
                        LOG("--- Cycle TX complete -------");
                    } else if ((systickGetMillis() - tx_ctx.tx_timestamp) >= UART_TIMEOUT_MS) 
                    {
                        if (tx_ctx.retry_count < RETRY_MAX) {
                            tx_ctx.state = STATE_SEND_DATA;
                            LOG("[0x%02X]DATA_ACK timeout. Retrying...", tx_ctx.seq);
                        } else {
                            tx_ctx.retry_count = 0;
                            tx_ctx.state = STATE_IDLE;
                            LOG("[0x%02X]DATA send failed after %d attempts. Going IDLE", tx_ctx.seq, RETRY_MAX);
                        }
                    }
                    break;
                case STATE_IDLE:
                    tx_ctx.seq = (tx_ctx.seq + 1) & 0x7F;           // Increment seq number
                    break;
            }
        }
    }
}

void uart_tx_set_flag(UART_PacketType_t type)
{
    switch (type)
    {
        case PKT_HANDSHAKE_ACK: tx_ctx.flag_handshake_ack = 1; break;
        case PKT_DATA_ACK:      tx_ctx.flag_data_ack      = 1; break;
        default: break;
    }
}

static void send_handshake_request(uint8_t seq)
{
    UART_Handshake_Packet_t pkt;

    pkt.sof       = SOF_BYTE;
    pkt.version   = PROTOCOL_VERSION;
    pkt.device_id = DEVICE_ID_STM32;
    pkt.seq_num   = seq;
    pkt.type      = PKT_HANDSHAKE_REQ;
    pkt.crc       = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);   // Exclude 3 last bytes (crc & eof)
    pkt.eof       = EOF_BYTE;

    LOG("[0x%02X] TX HANDSHAKE_REQ  : SOF:0x%02X VER:0x%02X DEV:0x%02X SEQ:0x%02X TYPE:0x%02X CRC:0x%02X 0x%02X EOF:0x%02X",
        tx_ctx.seq, pkt.sof, pkt.version, pkt.device_id, pkt.seq_num, pkt.type, pkt.crc >> 8, pkt.crc & 0x00FF, pkt.eof);
    
    uart1_write(pkt.sof);
    uart1_write(pkt.version);
    uart1_write(pkt.device_id);
    uart1_write(pkt.seq_num);
    uart1_write(pkt.type);
    uart1_write(pkt.crc >> 8);
    uart1_write(pkt.crc & 0x00FF);
    uart1_write(pkt.eof);
}

static void send_data_packet(uint8_t seq, const TXQueue_Item_t *pQueueItem)
{

    UART_Data_Packet_t pkt;

    pkt.sof       = SOF_BYTE;
    pkt.version   = PROTOCOL_VERSION;
    pkt.device_id = DEVICE_ID_STM32;
    pkt.seq_num   = seq;
    pkt.type      = PKT_DATA;
    pkt.length    = sizeof(TXQueue_Item_t);
    memcpy(pkt.payload, pQueueItem, pkt.length);
    pkt.crc       = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);   // Exclude 3 last bytes (crc & eof)
    pkt.eof       = EOF_BYTE;

    LOG("[0x%02X] TX DATA : SOF:0x%02X VER:0x%02X DEV:0x%02X SEQ:0x%02X  LEN:0x%02X PAYLOAD:0x%02X CRC:0x%02X 0x%02X EOF:0x%02X",
        tx_ctx.seq, pkt.sof, pkt.version, pkt.device_id, pkt.seq_num, pkt.type, pkt.length, pkt.crc >> 8, pkt.crc & 0x00FF, pkt.eof);
    
    uart1_write(pkt.sof);
    uart1_write(pkt.version);
    uart1_write(pkt.device_id);
    uart1_write(pkt.seq_num);
    uart1_write(pkt.type);
    uart1_write(pkt.length);
    for (uint8_t i=0; i < pkt.length; i++) {
        uart1_write(pkt.payload[i]);
    }
    uart1_write(pkt.crc >> 8);            // Shift right 8bits and send the high byte
    uart1_write(pkt.crc & 0x00FF);        // mask lower 8bits and send the low byte
    uart1_write(pkt.eof);
}