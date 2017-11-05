/*
 * uart_echo.c
 *
 *  Created on: 2016.04.20.
 *      Author: baadamff
 */

/* Board support header */
#include "bsp.h"

/* emlib & emdrv */
#include "em_int.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "uartdrv.h"
#include "uart_echo.h"
#include <stdio.h>
#include <string.h>



#define NUM_TX_BUFFERS          8

/***************************************************************************************************
 Local Variables
 **************************************************************************************************/

/***************************************************************************************************
 Public Function Definitions
 **************************************************************************************************/
void UART_Test(void)
{
//	char test_string[20];
//	strcpy(test_string, "SPP server\r\n");

	// More Test Code
	while(1)
	{
		// Send Test String to UART.
		UART_Send( "SPP server\r\n" );
	}

}

/**
 * @brief  This function Sends the passed string out via the UART Channel 1.
 * @param  char *test_string: String to be sent via UART. Should be terminated by a NULL/0x00.
 * @retval None
 */
void UART_Send(char *test_string)
{
	while(*test_string != (char)0x00)
	{
		USART_Tx (USART1, *test_string);
		test_string++;
	}
}

