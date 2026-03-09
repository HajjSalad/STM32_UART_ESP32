/**
 * @file  uart_router_task.c
 * @brief 
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>

#include "crc_16.h"
#include "priorities.h"
#include "uart_driver.h"
#include "uart_tx_task.h"
#include "uart_rx_task.h"
#include "uart_router_task.h"
#include "shared_resources.h"

static UART_Router_Context rt_ctx = {
    .buffer_index = 0,
    .len          = 0,
};

static void process_rx(const UART_Router_Context *pRtCtx)
{
    if (pRtCtx->buffer[SOF_POS]         != SOF_BYTE) return;         // Validate SoF
    if (pRtCtx->buffer[pRtCtx->len - 1] != EOF_BYTE) return;         // Validate EoF

    uint8_t type = pRtCtx->buffer[TYPE_POS];                        // Get packet type

    switch(type) 
    {
        case PKT_HANDSHAKE_REQ:                         // Goes to RX                 
        {
            RXQueue_Item_t handshake = {            
                .seq  = pRtCtx->buffer[SEQ_POS], 
                .type = pRtCtx->buffer[TYPE_POS],
            };
            xQueueSend(rx_queue, &handshake, 0);
            break;
        }
        case PKT_HANDSHAKE_ACK:                         // Goes to TX
        {
            uart_tx_set_flag(PKT_HANDSHAKE_ACK); 
            break;
        }
        case PKT_DATA:                                  // Goes to RX
        {
            RXQueue_Item_t data = {
                .seq  = pRtCtx->buffer[SEQ_POS], 
                .type = pRtCtx->buffer[TYPE_POS], 
            };
            memcpy(&data.payload, &pRtCtx->buffer[6], sizeof(MAX_PAYLOAD_SIZE));
            xQueueSend(rx_queue, &data, 0);
            break;
        }
        case PKT_COMMAND_ACK:                           // Goes to TX
        {
            uart_tx_set_flag(PKT_DATA_ACK);
            break;
        }
        default: 
            break;
    }
}

static void uart_router_task(void *pvParameters)
{
    uart_event_t event;

    while (1) 
    {
        // Block on UART queue waiting for RX from STM32
        if (xQueueReceive(uart_2_queue, (void *)&event, portMAX_DELAY))     // Block wait for item on queue
        {
            if (event.type == UART_PATTERN_DET)                             // trigger on pattern EOF_BYTE detection 
            {
                int pos = uart_pattern_pop_pos(UART_NUM2);                  // get position of the triggering byte, EOF_BYTE
                if (pos != -1) {
                    rt_ctx.len = pos + 1;                                   // record length all bytes including the EOF_BYTE

                    uart_read_bytes(UART_NUM2, rt_ctx.buffer, rt_ctx.len, portMAX_DELAY);
                    process_rx(&rt_ctx);
                }
            }
        }
    }
}

void uart_router_task_init(void)
{
    xTaskCreate(uart_router_task, "uart_router_task", 2048, NULL, TASK_PRIO_6, NULL);
}