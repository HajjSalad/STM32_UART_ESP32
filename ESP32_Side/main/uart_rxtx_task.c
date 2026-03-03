/**
 * @file uart_rxtx_task.c
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "string.h"
#include <stdint.h>
#include "stdio.h"

#include "uart.h"

volatile uint8_t flag_handshake_ack = 0;
volatile uint8_t flag_data_ack      = 0;
uint8_t seq = 0;

uint16_t compute_crc(uint8_t *data, uint8_t len)
{
    return 0xFFFF;    // TODO: implement CRC16
}

void process_rx(uint8_t *buf, uint8_t len)
{
    if (buf[0]     != 0xAA) return;         // Validate SoF
    if (buf[len-1] != 0x55) return;         // Validate EoF

    uint8_t type = buf[2];                  // Get packet type

    switch(type) 
    {
        case PKT_HANDSHAKE_REQ:
            flag_handshake_ack = 1;
            break;
        case PKT_DATA:
            flag_data_ack =1;
            break;
        default:
            break;
    }
}

void send_handshake_ack(uint8_t seq)
{
    UART_Handshake_Packet_t pkt;
    pkt.sof     = 0xAA;
    pkt.seq_num = seq;
    pkt.type    = PKT_HANDSHAKE_ACK;
    pkt.crc     = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);
    pkt.eof     = 0x55;

    uart_write_bytes(UART_NUM2, (uint8_t*)&pkt, sizeof(pkt));
}

void send_data_ack(uint8_t seq)
{
    UART_Handshake_Packet_t pkt;
    pkt.sof     = 0xAA;
    pkt.seq_num = seq;
    pkt.type    = PKT_DATA_ACK;
    pkt.crc     = compute_crc((uint8_t*)&pkt, sizeof(pkt) - 3);
    pkt.eof     = 0x55;

    uart_write_bytes(UART_NUM2, (uint8_t*)&pkt, sizeof(pkt));
}

static void rxtx_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t rx_data[RX_BUF_SIZE] = {0};

    while (1) {
        if (xQueueReceive(uart_2_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) 
            {
                // 1. Clear buffer before reading
                memset(rx_data, 0, sizeof(rx_data));

                // 2. Read data
                int len = (event.size < sizeof(rx_data)-1) ? event.size : sizeof(rx_data)-1;
                uart_read_bytes(UART_NUM2, rx_data, len, portMAX_DELAY);

                // 3. Process the received data
                process_rx(rx_data, len);

                // Print received data
                printf("Received: %s\n", rx_data);

                if (flag_handshake_ack) {
                    flag_handshake_ack = 0;
                    send_handshake_ack(seq);
                    printf("Handshake ACK sent\n");
                } 
                else if (flag_data_ack) {
                    flag_data_ack = 0;
                    send_data_ack(seq);
                    printf("Data ACK sent\n");
                }
                else {
                    printf("Unknown data received.\n\n");
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