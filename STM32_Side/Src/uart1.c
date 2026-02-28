/**
 * @file uart2.c
 * @brief UART drive implementation
 * 
 * Provides low-level UART1 initialization and transmit/receive functionality.
*/

#include "stm32f446xx.h"
#include "systick.h"
#include "uart.h"

#include <stdio.h>
#include <string.h>

#define GPIOAEN				(1U<<0)
#define UART1EN				(1U<<4)

#define CR1_TE				(1U<<3)
#define CR1_RE				(1U<<2)
#define CR1_UE				(1U<<13)
#define SR_TXE				(1U<<7)
#define SR_RXNE				(1U<<5)
#define SR_ORE              (1U<<3)

#define SYS_FREQ        	((uint32_t) 16000000)
#define APB2_CLK        	SYS_FREQ
#define UART_BAUDRATE   	((uint32_t) 115200)
#define UART_TIMEOUT_MS		1000U

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
	USART1->CR1 |= CR1_UE;				// Enable USART Module
}

/**
 * @brief Transmit a null-terminated string over UART1.
 * 
 * Appends a newline character after the string so the ESP32 can 
 * detect the end of the message.
 * 
 * @param str Pointer to a null-terminated string to transmit.
*/
void uart1_write_string(const char *str) 
{
	if(str == NULL) return;

    while (*str != '\0') 
	{
        uart1_write((int)*str);
        str++;
    }
    uart1_write('\n');  // Terminator the ESP32 can detect
}

/**
 * @brief Transmit a single byte over UART1.
 * 
 * Blocks until the transmit register is empty.
 * 
 * @param ch  Byte to transmit.
*/
void uart1_write(int ch) 
{
	while(!(USART1->SR & SR_TXE)){};		// Make sure the transmit data register is empty.
	USART1->DR = ((uint32_t)ch & 0xFF);		// Write to transmit data register
}

/**
 * @brief Receive a single character from UART1 with a 1s timeout.
 *
 * Checks and clears any overrun error before waiting for incoming data.
 *
 * @param ch  Pointer to store the received byte
 * 
 * @return 0 on success, -1 on failure
*/
int uart1_read(char *ch) 
{
	if (ch == NULL) return -1;

    // Check for overrun first and clear it
    if (USART1->SR & SR_ORE) {
        (void)USART1->SR;
        (void)USART1->DR;
    }

	uint32_t start = systickGetMillis();
    while(!(USART1->SR & SR_RXNE))
	{
		if ((systickGetMillis() - start) >= UART_TIMEOUT_MS)
		{
			return -1;		// Timeout
		}
	};		
	
	*ch = (char)(USART1->DR & 0xFF);
	return 0;
}

/**
 * @brief Read characters from UART1 into a buffer until '\n' or buffer full.
 * @param buf       Buffer to store received string.
 * @param max_len   Maximum number of bytes to read (including null terminator).
 * @return Number of characters read (excluding null terminator).
*/
uint16_t uart1_read_string(char *buf, uint16_t max_len)
{
    if (buf == NULL || max_len == 0) return 0;

    uint16_t i = 0;
	uint32_t start = systickGetMillis();

    while (i < (max_len - 1))
    {
        char c;
		if (uart1_read(&c) != 0)
			return -1;

        if (c == '\n' || c == '\r') break;
        buf[i++] = c;

		if ((systickGetMillis() - start) >= UART_TIMEOUT_MS)
			return -1;
    }

    buf[i] = '\0';
    return i;
}

/**
 * @brief Read a response from UART1 and compare it to an expected string.
 * @param expected  The string to compare against.
 * @return 0 if match, -1 if mismatch.
*/
int uart1_expect(const char *expected)
{
    if (expected == NULL) return -1;

    char rx_buf[32] = {0};

    if (uart1_read_string(rx_buf, sizeof(rx_buf)) < 0)
        return -1;  // Timeout, don't bother comparing

    return (strncmp(rx_buf, expected, strlen(expected)) == 0) ? 0 : -1;
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
