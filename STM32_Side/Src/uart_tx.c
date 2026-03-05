/**
 * @file  uart_tx.c
 * @brief 
*/

#include "stm32f446xx.h"
#include <string.h>

#include "crc_16.h"
#include "uart_tx.h"
#include "systick.h"
#include "uart_driver.h"
#include "shared_resources.h"

// String for testing
const uint8_t TX_DATA[]  = "Message from STM32";

// Instantiate a TX context
static UART_TX_Context tx_ctx = {
    .state       = STATE_SEND_HANDSHAKE,
    .seq         = SEQ_STM32_BASE,
    .retry_count = 0,
};

static void send_handshake_request(uint8_t seq)
{
    UART_Handshake_Packet_t pkt;

    pkt.sof     = SOF_BYTE;
    pkt.seq_num = seq;
    pkt.type    = PKT_HANDSHAKE_REQ;
    pkt.crc     = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);
    pkt.eof     = EOF_BYTE;

    LOG("[0x%02X] TX HANDSHAKE_REQ  : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
        tx_ctx.seq,
        pkt.sof,
        pkt.seq_num,
        pkt.type,
        pkt.crc >> 8,
        pkt.crc & 0xFF,
        pkt.eof);

    uart1_write(pkt.sof);
    uart1_write(pkt.seq_num);
    uart1_write(pkt.type);
    uart1_write(pkt.crc >> 8);
    uart1_write(pkt.crc & 0xFF);
    uart1_write(pkt.eof);
}

static void send_data_packet(uint8_t seq)
{
    UART_Data_Packet_t pkt;

    pkt.sof      = SOF_BYTE;
    pkt.seq_num  = seq;
    pkt.type     = PKT_DATA;
    pkt.length   = sizeof(TX_DATA) - 1;
    memcpy(pkt.payload, TX_DATA, pkt.length);
    pkt.crc      = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);
    pkt.eof      = EOF_BYTE;

    LOG("[0x%02X] TX DATA           : 0x%02X 0x%02X 0x%02X 0x%02X [%d payload bytes] 0x%02X 0x%02X 0x%02X",
        tx_ctx.seq,
        pkt.sof,
        pkt.seq_num,
        pkt.type,
        pkt.length,
        pkt.length,
        pkt.crc >> 8,
        pkt.crc & 0xFF,
        pkt.eof);

    uart1_write(pkt.sof);
    uart1_write(pkt.seq_num);
    uart1_write(pkt.type);
    uart1_write(pkt.length);
    for (uint8_t i=0; i < pkt.length; i++) {
        uart1_write(pkt.payload[i]);
    }
    uart1_write(pkt.crc >> 8);
    uart1_write(pkt.crc & 0xFF);
    uart1_write(pkt.eof);
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

void uart_tx_process(void)
{
    switch(tx_ctx.state) 
    {
        case STATE_SEND_HANDSHAKE:
            send_handshake_request(tx_ctx.seq);
            tx_ctx.tx_timestamp = systickGetMillis();       // Get timestamp
            tx_ctx.retry_count++;                           // Increment retry count
            tx_ctx.state = STATE_WAIT_HANDSHAKE_ACK;        // Move to wait for ACK
            break;
        case STATE_WAIT_HANDSHAKE_ACK:
            if (tx_ctx.flag_handshake_ack) {
                tx_ctx.flag_handshake_ack = 0;
                tx_ctx.retry_count = 0;
                tx_ctx.state = STATE_SEND_DATA;
                LOG("[0x%02X] RX HANDSHAKE_ACK  : Handshake ACK received", tx_ctx.seq);
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
            send_data_packet(tx_ctx.seq);
            tx_ctx.retry_count++;
            tx_ctx.tx_timestamp = systickGetMillis();
            tx_ctx.state = STATE_WAIT_DATA_ACK;
            break;
        case STATE_WAIT_DATA_ACK:
            if (tx_ctx.flag_data_ack) {
                tx_ctx.flag_data_ack = 0;
                tx_ctx.state = STATE_IDLE;
                LOG("[0x%02X] RX DATA_ACK       : Data ACK received", tx_ctx.seq);
                LOG("--- Cycle complete -------");
            } else if ((systickGetMillis() - tx_ctx.tx_timestamp) >= UART_TIMEOUT_MS) 
            {
                LOG("[0x%02X]Waiting for data ACK. Retry: %d", tx_ctx.seq, tx_ctx.retry_count);
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
            systickDelayMs(5000);
            tx_ctx.seq = (tx_ctx.seq + 1) & 0x7F;           // Increment seq number
            tx_ctx.retry_count = 0;
            tx_ctx.state = STATE_SEND_HANDSHAKE;
            LOG("\n--- Starting new cycle ---");
            break;
    }
}
        