/**
 * @file main.c
 * @brief Program entry point 
*/

#include "stm32f446xx.h"

#include <string.h>

#include "uart.h"
#include "systick.h"

#define   BUFFER_SIZE    32

const uint8_t TX_DATA[]  = "Message from STM32";
const uint8_t TX_READY[] = "READY?";

void send_to_ESP32()
{
    // Send READY message
    LOG("Send READY to ESP32...");
    uart1_write_string((const char*)TX_READY);

    // Wait for OK from ESP32
    if (uart1_expect("OK") == 0)
    {
        LOG("OK received from ESP32...");

        // Send data message
        LOG("Send Message to ESP32...");
        uart1_write_string((const char*)TX_DATA);

        // Wait for ACK from ESP32
        if (uart1_expect("ACK") == 0)
        {
            LOG("ACK received from ESP32...");
            LOG("Transmission complete\n");
        } else {
            LOG("Timeout waiting for ACK.\n");
            LOG("Unknown response received...");
        }
    } else {
        LOG("Timeout waiting for OK.\n");
    }
}

int main(void) 
{
    uart1_init();           // Initialize UART1
    uart2_init();           // Initialize UART2
    systick_init();         // Initialize SysTick

    LOG("*** Program start ***");

    while (1) 
    {
        send_to_ESP32();
        //LOG("Main Running");
        systickDelayMs(3000);
    }
}
