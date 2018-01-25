/* SiLabs BGM111 module access header file */

#ifndef BGM111A_H_
#define BGM111A_H_

#include "usart.h"
#include "stm32l1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

// Enums
typedef enum 
{
  TACK_OFF      = 0,
  TACK_ARMED    = 1,
  TACK_ARMED2   = 2,
  TACK_SYNC     = 3,
  TACK_ASYNC    = 4,
} Tack_state;

typedef enum
{
  SYNC_WAIT     = 0,
  SYNC_PROC     = 1,
} Sync_state;

/* BGM111 module interface pins and peripherals */
#define BGM111_UART                      USART3
#define BGM111_UART_CLK                  RCC_APB1Periph_USART3
#define BGM111_UART_GPIO_PORT            GPIOB
#define BGM111_UART_BAUDRATE             115200

#define BGM111_UART_TX_PIN               GPIO_Pin_10
#define BGM111_UART_TX_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define BGM111_UART_TX_SOURCE            GPIO_PinSource10
#define BGM111_UART_TX_AF                GPIO_AF_USART3

#define BGM111_UART_RX_PIN               GPIO_Pin_11
#define BGM111_UART_RX_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define BGM111_UART_RX_SOURCE            GPIO_PinSource11
#define BGM111_UART_RX_AF                GPIO_AF_USART3

#define BGM111_UART_CTS_PIN              GPIO_Pin_13
#define BGM111_UART_CTS_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define BGM111_UART_CTS_SOURCE           GPIO_PinSource13
#define BGM111_UART_CTS_AF               GPIO_AF_USART3

#define BGM111_UART_RTS_PIN              GPIO_Pin_14
#define BGM111_UART_RTS_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define BGM111_UART_RTS_SOURCE           GPIO_PinSource14
#define BGM111_UART_RTS_AF               GPIO_AF_USART3

#define	BGM111_UART_IRQn                 USART3_IRQn
#define	BGM111_UART_IRQHandler           USART3_IRQHandler

#define BGM111_RESET_GPIO_PORT           GPIOB
#define BGM111_RESET_PIN                 GPIO_Pin_15
#define BGM111_RESET_GPIO_CLK            RCC_AHBPeriph_GPIOB

#define TX_TIMEOUT_CNT                   2000           // Loop 2000 Times for Timeout
#define TACK_LIMIT                       80             // Set limit at 40 seconds before dropping as reset.
#define CMD_TIME                         10             // Set as a 1 Second Timer for Report.
#define RX_BFFR_MAX_LNGTH                256            // Controls the Maximum number of chars that can be tracked by Input buffer.
/* Initialize the BGM111 module and BGLib */
void BGM111_Init(void);

/* Process any input from the BLE module */
void BGM111_ProcessInput(void);

/* BLE write characteristic */
void BGM111_WriteCharacteristic(uint8_t handle, uint8_t len, uint8_t *data);
HAL_StatusTypeDef SkyBrd_ProcessBGMChar(uint8_t c);
bool SkyBrd_tstReqexec( void );
//void BGM111_UART_IRQHandler(UART_HandleTypeDef *huart);
//uint16_t USART_ReceiveData(UART_HandleTypeDef *huart);
//void USART_SendData(UART_HandleTypeDef *huart, uint16_t Data);
void BGM111_Transmit(uint32_t len, uint8_t *data);
uint8_t BGM111_GetTackState(void);
void BGM111_SetTackState(uint8_t NewValue);
bool BGM111_SyncModeTest(void);
void BGM111_SetSyncFlg(uint8_t NewFlag);
void BGM111_cntrlSetSyncFlg(uint8_t NewFlag);
bool BGM111_SyncModeTestNoInc(void);

/* Report if the BLE is ready for a command */
bool BGM111_Ready(void);
bool BGM111_Connected(void);
bool BGM111_DataConnected(void);
void BGM111_SetCMD_Mode(bool NewMode);
bool BGM111_CMD_Mode(void);
void BGM111_SetDataConnected(bool NewMode);
HAL_StatusTypeDef set_BGMBanner( char *string1 );
char *get_BGMBanner( void );

#endif