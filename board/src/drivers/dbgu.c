/*
 * dbgu.c
 *	Source file where debug usart functions implemented
 *  Created on: Jul 23, 2019
 *      Author: michael
 */
#include "state.h"
#if (DBGU)

#include <stdint.h>
#include "xprintf.h"
#include "stm32f4xx_hal_usart.h"


void dbgu_write(uint8_t symbol)
{
	HAL_USART_Transmit(&usart_dbg, &symbol, 1, 1);
}


//	FIXME: BAD
uint8_t dbgu_read()
{
	uint8_t symbol = 0;
	HAL_USART_Receive(&usart_dbg, &symbol, 1, 1);
	return symbol;
}


void _init_usart_dbg()
{
	//	USART module
	usart_dbg.Instance = USART1;
	usart_dbg.Init.BaudRate = 115200;
	usart_dbg.Init.WordLength = UART_WORDLENGTH_8B;
	usart_dbg.Init.StopBits = UART_STOPBITS_1;
	usart_dbg.Init.Parity = UART_PARITY_NONE;
	usart_dbg.Init.Mode = UART_MODE_TX_RX;
	HAL_USART_Init(&usart_dbg);

	//	Setting up xprintf
	xdev_out(dbgu_write);
	xdev_in(dbgu_read);
}

#endif
