/**
 * @file  uart_tx.c
 * @brief 
*/

#include "stm32f446xx.h"
#include <string.h>

#include "uart_tx.h"
#include "shared_resources.h"

// String for testing
const uint8_t TX_DATA[]  = "Message from STM32";

// Instantiate a TX context
static UART_TX_Context tx_ctx = {
    .state       = STATE_SEND_HANDSHAKE,
    .seq         = SEQ_STM32_BASE,
    .retry_count = 0,
};

void send_handshake_request(uint8_t seq)
{
    UART_Handshake_Packet_t pkt;
    pkt.sof     = SOF_BYTE;
    pkt.seq_num = seq;
    pkt.type    = PKT_HANDSHAKE_REQ;
    pkt.crc     = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);
    pkt.eof     = EOF_BYTE;

    uart1_write(pkt.sof);
    uart1_write(pkt.seq_num);
    uart1_write(pkt.type);
    uart1_write(pkt.crc >> 8);
    uart1_write(pkt.crc & 0xFF);
    uart1_write(pkt.eof);
}

void send_data_packet(uint8_t seq)
{
    UART_Data_Packet_t pkt;
    pkt.sof      = SOF_BYTE;
    pkt.seq_num  = seq;
    pkt.length   = sizeof(TX_DATA) -1;
    pkt.type     = PKT_DATA;
    memcpy(pkt.payload, TX_DATA, pkt.length);
    pkt.crc      = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);
    pkt.eof      = EOF_BYTE;

    uart1_write(pkt.sof);
    uart1_write(pkt.seq_num);
    uart1_write(pkt.length);
    uart1_write(pkt.type);
    for (uint8_t i=0; i < pkt.length; i++) {
        uart1_write(pkt.payload[i]);
    }
    uart1_write(pkt.crc >> 8);
    uart1_write(pkt.crc & 0xFF);
    uart1_write(pkt.eof);
}

void uart_tx_set_flag(uint8_t flag_type)
{
    
}

void uart_tx_process(void)
{
    switch(tx_ctx.state) 
    {
        case STATE_SEND_HANDSHAKE:
            send_handshake_request(tx_ctx.seq);
            tx_ctx.seq = (tx_ctx.seq + 1) & 0x7F;           // Increment seq number
            tx_ctx.tx_timestamp = systickGetMillis();       // Get timestamp
            tx_ctx.retry_count++;                           // Increment retry count
            tx_ctx.state = STATE_WAIT_HANDSHAKE_ACK;        // Move to wait for ACK
            break;
        case STATE_WAIT_HANDSHAKE_ACK:
            if (flag_handshake_ack) {
                flag_handshake_ack = 0;
                retry_count = 0;
                state = STATE_SEND_DATA;
            }
            else if ((systickGetMillis() - tx_timestamp) >= UART_TIMEOUT_MS)  {
                if (retry_count < RETRY_MAX) {
                    state = STATE_SEND_HANDSHAKE;
                } else {
                    state = STATE_IDLE;
                    LOG("Handshake failed after %d retries", retry_count);
                    retry_count = 0;
                }
            }
            break;
        case STATE_SEND_DATA:
            send_data_packet(seq++);
            retry_count++;
            tx_timestamp = systickGetMillis();
            state = STATE_WAIT_DATA_ACK;
            break;
        case STATE_WAIT_DATA_ACK:
            if (flag_data_ack) {
                flag_data_ack = 0;
                state = STATE_IDLE;
            } else if ((systickGetMillis() - tx_timestamp) >= UART_TIMEOUT_MS) {
                if (retry_count < RETRY_MAX) {
                    state = STATE_SEND_DATA;
                } else {
                    state = STATE_IDLE;
                    LOG("Send data failed after %d retries", retry_count);
                    retry_count = 0;
                }
            }
            break;
        case STATE_IDLE:
            systickDelayMs(3000);
            seq = 0;
            retry_count = 0;
            state = STATE_SEND_HANDSHAKE;
            break;
    }
}
// Process incoming bytes
        if (rx_ready) {
            rx_ready = 0;
            process_rx(rx_buffer, rx_index);
        }

        