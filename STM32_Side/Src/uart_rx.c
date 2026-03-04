/**
 * @file  uart_rx.c
 * @brief  
*/

#include "stm32f446xx.h"
#include <string.h>

#include "uart_rx.h"
#include "uart_tx.h"
#include "shared_resources.h"

static UART_RX_Context rx_ctx = {
    .buffer_index = 0,
};

void process_rx(uint8_t *buf, uint8_t len) 
{
    if (buf[0]     != SOF_BYTE) return;         // Validate SOF
    if (buf[len-1] != EOF_BYTE) return;         // Validate EOF

    uint8_t type = buf[2];                      // Get type of packet

    switch(type) 
    {
        case PKT_HANDSHAKE_ACK: 
            uart_tx_set_flag(PKT_HANDSHAKE_ACK); 
            break;
        case PKT_DATA_ACK:      
            uart_tx_set_flag(PKT_DATA_ACK);
            break;
        default: 
            break;
    }
}

// Gets byte from ISR. Check if its SOF_BYTE, start recording the packet.
// Keep checking the byte. If its EOF_BYTE, call process_rx and reset the rx_buffer
void uart_rx_push_byte(uint8_t byte)
{
    if (byte == SOF_BYTE) {
        rx_ctx.buffer_index = 0;                                    // reset on new packet
        rx_ctx.rx_buffer[rx_ctx.buffer_index++] = byte;             // Starting recording packets
    }
    else if (rx_ctx.buffer_index > 0) {                             // only record if SOF seen
        rx_ctx.rx_buffer[rx_ctx.buffer_index++] = byte;

        if (byte == EOF_BYTE) {
            process_rx(rx_ctx.rx_buffer, rx_ctx.buffer_index);
            rx_ctx.buffer_index = 0;                                 // reset for next packet
        }
    }
}