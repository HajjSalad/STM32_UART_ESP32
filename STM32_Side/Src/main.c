/**
 * @file  main.c
 * @brief Program entry point 
*/

#include "stm32f446xx.h"
#include <string.h>
#include "systick.h"
#include "uart_tx.h"
#include "uart_rx.h"
#include "uart_driver.h"
#include "shared_resources.h"

int main(void) 
{
    uart1_init();               // Initialize UART1
    uart2_init();               // Initialize UART2
    systick_init();             // Initialize SysTick

    LOG("*** Program start ***");

    while (1) 
    {
        uart_tx_process();      // drive TX state machine
    }
}
