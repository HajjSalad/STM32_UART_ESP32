/**
 * @file  main.c
 * @brief Program entry point 
*/

#include "stm32f446xx.h"
#include <string.h>
#include "uart.h"
#include "systick.h"


int main(void) 
{
    uart1_init();               // Initialize UART1
    uart2_init();               // Initialize UART2
    systick_init();             // Initialize SysTick

    LOG("*** Program start ***");

    while (1) 
    {
        uart_tx_process();      // drive TX state machine
        uart_rx_process();      // drive RX state machine
    }
}
