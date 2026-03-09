/**
 * @file  uart_rx_task.c
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
#include "priorities.h"
#include "uart_driver.h"
#include "uart_rx_task.h"
#include "shared_resources.h"

static const char *TAG = "TX";

// Static Funbction Prototypes
static void send_handshake_ack(uint8_t seq);
static void send_data_ack(uint8_t seq);

static void uart_rx_task(void *pvParameters)
{
    RXQueue_Item_t rx_data = {0};

    while(1) 
    {
        if (xQueueReceive(rx_queue, &rx_data, portMAX_DELAY)) 
        {
            switch(rx_data.type)
            {
                case PKT_HANDSHAKE_REQ:
                    printf("\n--- Starting new RX cycle ---\n");
                    printf("[0x%02X] RX HANDSHAKE_REQ  : received\n", rx_data.seq);
                    send_handshake_ack(rx_data.seq);
                    printf("[0x%02X] TX HANDSHAKE_ACK  : sent\n", rx_data.seq);
                    break;
                case PKT_DATA:
                    printf("[0x%02X] RX DATA           : received\n", rx_data.seq);
                    send_data_ack(rx_data.seq);
                    printf("[0x%02X] TX DATA_ACK       : sent\n", rx_data.seq);
                    printf("--- Cycle RX complete -------\n\n");
                    break;

                default:
                     printf("[0x%02X] RX UNKNOWN type:received\n", rx_data.seq);
                    break;
            }
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

    uart_write_bytes(UART_NUM2, (uint8_t*)&pkt, sizeof(pkt));

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

    uart_write_bytes(UART_NUM2, (uint8_t*)&pkt, sizeof(pkt));

    printf("[0x%02X] TX DATA_ACK       : 0x%02X 0x%02X 0x%02X [Payload] 0x%02X 0x%02X 0x%02X\n", 
        seq,
        pkt.sof,
        pkt.seq_num,
        pkt.type,
        pkt.crc >> 8,
        pkt.crc & 0xFF,
        pkt.eof
    );
}

void uart_rx_task_init(void)
{
    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, TASK_PRIO_5, NULL);
}