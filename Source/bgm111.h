/* SiLabs BGM111 module access header file */

#ifndef BGM111_H_
#define BGM111_H_

#include "stm32l1xx.h"
#include <stdint.h>
#include <stdbool.h>


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


/* Initialize the BGM111 module and BGLib */
void BGM111_Init(void);

/* Process any input from the BLE module */
void BGM111_ProcessInput(void);

/* BLE write characteristic */
void BGM111_WriteCharacteristic(uint8_t handle, uint8_t len, uint8_t *data);

/* Report if the BLE is ready for a command */
bool BGM111_Ready(void);
bool BGM111_Connected(void);

#endif