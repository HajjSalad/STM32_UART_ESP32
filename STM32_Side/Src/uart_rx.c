/**
 * @file  uart_rx.c
 * @brief  
*/

#include "stm32f446xx.h"
#include <string.h>

#include "uart_rx.h"
#include "shared_resources.h"


// Gets byte from ISR and process it
void uart_rx_push_byte(uint8_t byte)
{
    
}

void process_rx(uint8_t *buf, uint8_t len) 
{
    if (buf[0]     != SOF_BYTE) return;         // Validate SOF
    if (buf[len-1] != EOF_BYTE) return;         // Validate EOF

    uint8_t type = buf[2];                  // Get type of packet

    switch(type) 
    {
        case PKT_HANDSHAKE_ACK: flag_handshake_ack = 1; break;
        case PKT_DATA_ACK:      flag_data_ack      = 1; break;
        default: break;
    }

    rx_index = 0;       // reset buffer for next pocket
}