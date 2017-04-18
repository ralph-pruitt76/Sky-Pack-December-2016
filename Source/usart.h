/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 WeatherCloud
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include <stdint.h>
#include <stdbool.h>

/* USER CODE BEGIN Includes */
#include "bgm111.h"
#include "app_data.h"

/**************************** enums *******************************************/

/** 
  * @brief  HAL Status structures definition  
  */  
typedef enum 
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

/** 
  * @brief HAL UART State structures definition  
  */ 
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00,    /*!< Peripheral is not initialized                      */
  HAL_UART_STATE_READY             = 0x01,    /*!< Peripheral Initialized and ready for use           */
  HAL_UART_STATE_BUSY              = 0x02,    /*!< an internal process is ongoing                     */
  HAL_UART_STATE_BUSY_TX           = 0x12,    /*!< Data Transmission process is ongoing               */
  HAL_UART_STATE_BUSY_RX           = 0x22,    /*!< Data Reception process is ongoing                  */
  HAL_UART_STATE_BUSY_TX_RX        = 0x32,    /*!< Data Transmission and Reception process is ongoing */
  HAL_UART_STATE_TIMEOUT           = 0x03,    /*!< Timeout state                                      */
  HAL_UART_STATE_ERROR             = 0x04     /*!< Error                                              */
}HAL_UART_StateTypeDef;

/*************************** Defines ******************************************/

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                    80
/* Size of Reception buffer */
#define RXBUFFERSIZE                    TXBUFFERSIZE
// Maximum Delay for a Transfer.....30 Seconds
#define HAL_MAX_DELAY                   300000

#define MNTR_UART                      	USART1
#define MNTR_UART_CLK                  	RCC_APB2ENR_USART1EN
#define MNTR_UART_GPIO_PORT            	GPIOA
#define MNTR_UART_BAUDRATE             	19200
	
#define MNTR_UART_TX_PIN               	GPIO_Pin_9
#define MNTR_UART_TX_GPIO_CLK          	RCC_AHBPeriph_GPIOA
#define MNTR_UART_TX_SOURCE            	GPIO_PinSource9
#define MNTR_UART_TX_AF                	GPIO_AF_USART1
	
#define MNTR_UART_RX_PIN               	GPIO_Pin_10
#define MNTR_UART_RX_GPIO_CLK          	RCC_AHBPeriph_GPIOA
#define MNTR_UART_RX_SOURCE            	GPIO_PinSource10
#define MNTR_UART_RX_AF                	GPIO_AF_USART1
	
#define	MNTR_UART_IRQn                 	USART1_IRQn
#define	MNTR_UART_IRQHandler           	USART1_IRQHandler

/*************************** Prototypes ***************************************/

void MX_MNTR_UART_Init(void);
HAL_StatusTypeDef MNTR_UART_Transmit( uint8_t *pData, uint16_t Size, uint32_t Timeout );
void delay_100us( void );
HAL_StatusTypeDef SkyPack_MNTR_UART_Transmit( uint8_t *pData );
HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout( uint32_t Timeout );
bool Mntr_Cmd( void );
void Mntr_Clr( void );
void getMntrCmd(uint8_t *data);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
