/**
 * @file uart1.c
 * @brief UART1 driver implementation
 * 
 * Provides low-level UART1 initialization and transmit/receive functionality.
*/

#include "stm32f446xx.h"
#include "systick.h"
#include "uart_rx.h"
#include "uart_driver.h"

#include <stdio.h>
#include <string.h>

#define GPIOAEN				(1U<<0)
#define UART1EN				(1U<<4)

#define CR1_TE				(1U<<3)
#define CR1_RE				(1U<<2)
#define CR1_UE				(1U<<13)
#define CR1_RXNE			(1U<<5)
#define SR_TXE				(1U<<7)
#define SR_ORE              (1U<<3)

#define SYS_FREQ        	((uint32_t) 16000000)
#define APB2_CLK        	SYS_FREQ
#define UART_BAUDRATE   	((uint32_t) 115200)

// Function Prototypes
static void 	uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);

/**
 * @brief Initialize UART1 peripheral.
 * 
 * UART1 is configured for communication with ESP32
*/
void uart1_init(void) 
{
	RCC->AHB1ENR |= GPIOAEN;			// Enable clock GPIOA
    RCC->APB2ENR |= UART1EN;			// Enable clock to UART1

	GPIOA->MODER &=~(1U<<18);			// PA9 to alternate function mode
	GPIOA->MODER |= (1U<<19);
	GPIOA->AFR[1] |= (7U<<4);			// Set PA9 AF to UART1_TX (AF07)
    GPIOA->OSPEEDR |= (3<<18);			// High Speed for PA9

	GPIOA->MODER &=~(1U<<20);			// PA10 to alternate function mode
	GPIOA->MODER |= (1U<<21);
	GPIOA->AFR[1] |= (7U<<8);			// Set PA10 AF to UART1_RX (AF07)
    GPIOA->OSPEEDR |= (3<<20);			// High Speed for PA10
	
    USART1->CR1 = 0x00;   				// Clear ALL

	// Configure baudrate USART1
	uart_set_baudrate(USART1, APB2_CLK, UART_BAUDRATE);

	USART1->CR1 = (CR1_TE | CR1_RE);	// Configure the transfer direction
	USART1->CR1 |= CR1_RXNE;       		// Enable RXNE interrupt
	NVIC_EnableIRQ(USART1_IRQn);     	// Enable USART1 in NVIC
	USART1->CR1 |= CR1_UE;				// Enable USART Module
}

/**
 * @brief Transmit a single byte over UART1.
 * 
 * Blocks until the transmit register is empty.
 * 
 * @param byte  Byte to transmit.
*/
void uart1_write(uint8_t byte) 
{
	while(!(USART1->SR & SR_TXE)){};	// Make sure the transmit data register is empty.
	USART1->DR = (byte);				// Write to transmit data register
}													

/**
 * @brief USART1 interrupt service routine
 * 
 * Fires on every received byte.
 * Reads the byte from the data register and hands it off to RX layer for processing.
*/
void USART1_IRQHandler(void)
{
	if (USART1->SR & CR1_RXNE) {
		uart_rx_push_byte((uint8_t)(USART1->DR & 0xFF));
	}
}

/** @brief Configure the baud rate for the USART peripheral */
static void uart_set_baudrate(USART_TypeDef *USARTx, 
							  uint32_t PeriphClk, 
							  uint32_t BaudRate) 
{
	USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}

/** @brief Compute USART baud rate register (BRR) value */
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate) {

	return ((PeriphClk + (BaudRate / 2U)) / BaudRate);
}
