/**
 * @file  uart_rx_task.c
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
#include "uart_rx_task.h"
#include "shared_resources.h"

// Static function prototypes
static void send_handshake_ack(uint8_t seq);
static void send_data_ack(uint8_t seq);

// static UART_RX_Context rx_ctx = {
//     .state       = STATE_RECEIVE_HANDSHAKE,
//     .seq         = 0,
//     .retry_count = 0,
// };

void vTaskRX(void *pvParameters)
{
    (void)pvParameters;                 // Suppress unused parameter warning

    BaseType_t    xRet          = pdFALSE;
    RXQueue_Item_t queueItem    = {0U};

    while(1)
    {
        // Block here until data is available to send
        xRet = xQueueReceive(xRXQueue, &queueItem, portMAX_DELAY);
        if (xRet != pdTRUE) {
            LOG("RX queue receive failed unexpectedly");
            continue;
        }
        
        switch(queueItem.type)
        {
            case PKT_HANDSHAKE_REQ:
                LOG("\n--- Starting new cycle ---");
                LOG("[0x%02X] RX HANDSHAKE_REQ  : received", queueItem.seq);
                send_handshake_ack(queueItem.seq);
                LOG("[0x%02X] TX HANDSHAKE_ACK  : sent", queueItem.seq);
                break;

            case PKT_DATA:
                LOG("[0x%02X] RX DATA           : CMD", queueItem.seq);
                send_data_ack(queueItem.seq);
                LOG("[0x%02X] TX DATA_ACK       : sent", queueItem.seq);
                LOG("--- Cycle complete -------");
                break;

            default:
                LOG("[0x%02X] RX UNKNOWN type:0x%02X", queueItem.seq, queueItem.type);
                break;
        }
    }
}

static void send_handshake_ack(uint8_t seq)
{
    UART_ACK_Packet_t pkt;

    pkt.sof     = SOF_BYTE;
    pkt.seq_num = seq;
    pkt.type    = PKT_HANDSHAKE_ACK;
    pkt.crc     = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);
    pkt.eof     = EOF_BYTE;

    uart1_write(pkt.sof);
    uart1_write(pkt.seq_num);
    uart1_write(pkt.type);
    uart1_write(pkt.crc >> 8);
    uart1_write(pkt.crc & 0x00FF);
    uart1_write(pkt.eof);

    printf("[0x%02X] TX HANDSHAKE_ACK  : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
        seq,
        pkt.sof,
        pkt.seq_num,
        pkt.type,
        pkt.crc >> 8,
        pkt.crc & 0xFF,
        pkt.eof
    );
}

static void send_data_ack(uint8_t seq)
{
    UART_ACK_Packet_t pkt;

    pkt.sof     = SOF_BYTE;
    pkt.seq_num = seq;
    pkt.type    = PKT_DATA_ACK;
    pkt.crc     = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);
    pkt.eof     = EOF_BYTE;

    uart1_write(pkt.sof);
    uart1_write(pkt.seq_num);
    uart1_write(pkt.type);
    uart1_write(pkt.crc >> 8);
    uart1_write(pkt.crc & 0x00FF);
    uart1_write(pkt.eof);

    printf("[0x%02X] TX DATA_ACK       : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
        seq,
        pkt.sof,
        pkt.seq_num,
        pkt.type,
        pkt.crc >> 8,
        pkt.crc & 0xFF,
        pkt.eof
    );
}


