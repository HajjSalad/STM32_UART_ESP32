/**
 * @file  uart_rxtx_task.c
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "string.h"
#include <stdint.h>
#include "stdio.h"

#include "crc_16.h"
#include "uart_driver.h"
#include "uart_rxtx_task.h"

static UART_RX_Context rx_ctx = {
    .seq                 = 0,
    .flag_handshake_ack  = 0,
    .flag_data_ack       = 0,
};

static void process_rx(uint8_t *buf, uint8_t len)
{
    if (buf[0]     != SOF_BYTE) return;         // Validate SoF
    if (buf[len-1] != EOF_BYTE) return;         // Validate EoF

    rx_ctx.seq   = buf[1];                      // Get seq number
    uint8_t type = buf[2];                      // Get packet type

    switch(type) 
    {
        case PKT_HANDSHAKE_REQ:
            rx_ctx.flag_handshake_ack = 1;
            break;
        case PKT_DATA:
            rx_ctx.flag_data_ack =1;
            break;
        default:
            break;
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

    uart_write_bytes(UART_NUM2, (uint8_t*)&pkt, sizeof(pkt));

    printf("[0x%02X] TX HANDSHAKE_ACK  : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
        rx_ctx.seq,
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

    uart_write_bytes(UART_NUM2, (uint8_t*)&pkt, sizeof(pkt));

    printf("[0x%02X] TX DATA_ACK       : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
        rx_ctx.seq,
        pkt.sof,
        pkt.seq_num,
        pkt.type,
        pkt.crc >> 8,
        pkt.crc & 0xFF,
        pkt.eof
    );
}

static void rxtx_task(void *pvParameters)
{
    uart_event_t event;

    while (1) {
        if (xQueueReceive(uart_2_queue, (void *)&event, portMAX_DELAY))     // Block wait for item on queue
        {
            if (event.type == UART_PATTERN_DET)                             // trigger on pattern EOF_BYTE detection 
            {
                int pos = uart_pattern_pop_pos(UART_NUM2);                  // get position of the triggering byte, EOF_BYTE
                if (pos != -1) {
                    uint8_t len = pos + 1;                                  // all bytes including the EOF_BYTE
                    uint8_t buf[RX_BUF_SIZE] = {0};

                    uart_read_bytes(UART_NUM2, buf, len, portMAX_DELAY);
                    process_rx(buf, len);

                    if (rx_ctx.flag_handshake_ack) {
                        rx_ctx.flag_handshake_ack = 0;
                        printf("--- Starting new cycle ---\n");
                        printf("[0x%02X] RX HANDSHAKE_REQ  : ", rx_ctx.seq);
                        for (int i = 0; i < len; i++) printf("0x%02X ", buf[i]);
                        printf("\n");
                        send_handshake_ack(rx_ctx.seq);
                    }
                    else if (rx_ctx.flag_data_ack) {
                        rx_ctx.flag_data_ack = 0;
                        uint8_t payload_len = buf[3];
                        printf("[0x%02X] RX DATA           : 0x%02X 0x%02X 0x%02X 0x%02X [%d payload bytes] 0x%02X 0x%02X 0x%02X\n",
                            rx_ctx.seq,buf[0],buf[1],buf[2],buf[3],payload_len,buf[4 + payload_len],
                            buf[4 + payload_len + 1],buf[4 + payload_len + 2]);
                        send_data_ack(rx_ctx.seq);
                        printf("--- Cycle complete -------\n\n");
                    }
                    else {
                        printf("Unknown packet received:\n");
                        for (int i = 0; i < len; i++) printf("0x%02X ", buf[i]);
                        printf("\n");
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void uart_rxtx_task_init(void)
{
    xTaskCreate(rxtx_task, "uart_rxtx_task", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}