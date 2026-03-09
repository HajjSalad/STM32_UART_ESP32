/**
 * @file  uart_router.c
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
#include "uart_router.h"
#include "uart_driver.h"
#include "uart_tx_task.h"
#include "uart_rx_task.h"
#include "shared_resources.h"

static UART_Router_Context rt_ctx = {
    .buffer_index = 0,
};

static void process_rx(uint8_t *buf, uint8_t len) 
{
    if (buf[0]     != SOF_BYTE) return;         // Validate SOF
    if (buf[len-1] != EOF_BYTE) return;         // Validate EOF

    uint8_t seq  = buf[3];                      // Get seq number
    uint8_t type = buf[4];                      // Get type of packet

    switch(type) 
    {
        case PKT_HANDSHAKE_REQ:                     
        {
            RXQueue_Item_t queueItem = {
                .seq = buf[3], 
                .type = buf[4],
            };
            xQueueSendFromISR(xRXQueue, &queueItem, 0);
            break;
        }
        case PKT_HANDSHAKE_ACK:
        {
            uart_tx_set_flag(PKT_HANDSHAKE_ACK); 
            break;
        }
        case PKT_DATA:
        {
            RXQueue_Item_t queueItem = {
                .seq = buf[3], 
                .type = buf[4], 
                .code = buf[6],
            };
            xQueueSendFromISR(xRXQueue, &queueItem, 0);
            break;
        }
        case PKT_DATA_ACK:
        {
            uart_tx_set_flag(PKT_DATA_ACK);
            break;
        }
        default: 
            break;
    }
}

// Gets byte from ISR. Check if its SOF_BYTE, start recording the packet.
// Keep checking the byte. If its EOF_BYTE, call process_rx and reset the buffer
void uart_isr_push_byte(uint8_t byte)
{
    if (byte == SOF_BYTE) {
        rt_ctx.buffer_index = 0;                                    // reset on new packet
        rt_ctx.buffer[rt_ctx.buffer_index++] = byte;             // Starting recording packets
    }
    else if (rt_ctx.buffer_index > 0) {                             // only record if SOF seen
        rt_ctx.buffer[rt_ctx.buffer_index++] = byte;

        if (byte == EOF_BYTE) {
            process_rx(rt_ctx.buffer, rt_ctx.buffer_index);
            rt_ctx.buffer_index = 0;                                 // reset for next packet
        }
    }
}

