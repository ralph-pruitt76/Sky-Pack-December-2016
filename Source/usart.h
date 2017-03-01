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

/*************************** Defines ******************************************/

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                    80
/* Size of Reception buffer */
#define RXBUFFERSIZE                    TXBUFFERSIZE

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
#if 0
   
typedef enum 
{
  USART_2 = 0,
  USART_3 = 1,

  NUCLEO_USART = USART_2,
  ROADBRD_USART = USART_3,
  BGM_USART = USART_3,
//  BGM_USART = USART_2,
#ifdef PATCH_UART
  MONITOR_UART = USART_3
#else
  MONITOR_UART = USART_2
//  MONITOR_UART = USART_3
#endif
} RoadBrd_uart_TypeDef;

/* USER CODE END Includes */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */


/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef RoadBrd_UART_Transmit_IT(RoadBrd_uart_TypeDef Port, uint8_t *pData);
HAL_StatusTypeDef RoadBrd_UART_Transmit_ITSZ(RoadBrd_uart_TypeDef Port, uint8_t *pData, uint16_t Size);
ITStatus RoadBrd_Uart_Status(RoadBrd_uart_TypeDef Port);
#ifndef PATCH_UART
  HAL_StatusTypeDef RoadBrd_UART_Receive_ITBG(RoadBrd_uart_TypeDef Port, uint16_t Size);
  uint8_t *RoadBrd_GetBGPtr( void );
  void RoadBrd_SetBffrFlg( void );
  void RoadBrd_ClrBffrFlg( void );
  bool RoadBrd_GetBffrFlg( void );
  int RoadBrd_getSaveLen( void );
  void HAL_UART_EnableBGM_TXE( void );
  void HAL_UART_EnableBGM_RX( void );
#endif
HAL_StatusTypeDef RoadBrd_UART_Receive_IT(RoadBrd_uart_TypeDef Port, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef RoadBrd_UART_Receive(RoadBrd_uart_TypeDef Port, uint8_t *pData, uint16_t Size);
HAL_UART_StateTypeDef RoadBrd_Uart_GetState(RoadBrd_uart_TypeDef Port);
HAL_StatusTypeDef RoadBrd_UART_Transmit(RoadBrd_uart_TypeDef Port, uint8_t *pData);
void clrUsartState( RoadBrd_uart_TypeDef Port );

/* USER CODE END Prototypes */
#endif

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
