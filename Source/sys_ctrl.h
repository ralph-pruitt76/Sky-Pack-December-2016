/* System control header file
 * Drivers to control the capacitive sensing driven shield, the 
 * charging chip, the charging heater and the status LED */

#ifndef SYS_CTRL_H_
#define SYS_CTRL_H_

#include "stm32l1xx.h"
#include <stdint.h>
#include <stdbool.h>


/* Uncomment this for LED PWM driver */
/* #define PWM_LED */

/* Definitions for USB power */

enum eUSBPower
{
  USB_POWER_OFF,
  USB_POWER_CHARGE,
  USB_POWER_HEAT,
  
  USB_POWER_COUNT
};

/* System control interface pins and peripherals */

#define CHARGE_ON_GPIO_PORT             GPIOA
#define CHARGE_ON_PIN                   GPIO_Pin_7
#define CHARGE_ON_GPIO_CLK              RCC_AHBPeriph_GPIOA

#define HEAT_ON_GPIO_PORT               GPIOA
#define HEAT_ON_PIN                     GPIO_Pin_8
#define HEAT_ON_GPIO_CLK                RCC_AHBPeriph_GPIOA

#define SHIELD_ON_GPIO_PORT             GPIOA
#define SHIELD_ON_PIN                   GPIO_Pin_5
#define SHIELD_ON_GPIO_CLK              RCC_AHBPeriph_GPIOA

#define CHARGE_ON_GPIO_PORT             GPIOA
#define CHARGE_ON_PIN                   GPIO_Pin_7
#define CHARGE_ON_GPIO_CLK              RCC_AHBPeriph_GPIOA

#define HEAT_ON_GPIO_PORT               GPIOA
#define HEAT_ON_PIN                     GPIO_Pin_8
#define HEAT_ON_GPIO_CLK                RCC_AHBPeriph_GPIOA

#define VDD_ON_GPIO_PORT                GPIOA
#define VDD_ON_PIN                      GPIO_Pin_0
#define VDD_ON_GPIO_CLK                 RCC_AHBPeriph_GPIOA

#define BGM_ON_GPIO_PORT                GPIOA
#define BGM_ON_PIN                      GPIO_Pin_1
#define BGM_ON_GPIO_CLK                 RCC_AHBPeriph_GPIOA

#define RESET_BGM111_PIN                GPIO_Pin_15
#define RESET_BGM111_GPIO_PORT          GPIOB
#define RESET_BGM111_GPIO_CLK           RCC_AHBPeriph_GPIOB

#define STATUS_LED_GPIO_PORT            GPIOB
#define STATUS_LED_PIN                  GPIO_Pin_9
#define BLUE_PIN                        GPIO_Pin_9
#define BLUE_GPIO_PORT                  GPIOB
#define GREEN_PIN                       GPIO_Pin_12
#define GREEN_GPIO_PORT                 GPIOB
#define YELLOW_PIN                      GPIO_Pin_8
#define YELLOW_GPIO_PORT                GPIOB
#define STATUS_LED_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define STATUS_LED_SOURCE               GPIO_PinSource9
#define STATUS_LED_AF                   GPIO_AF_TIM11
#define STATUS_LED_TIM                  TIM11
#define STATUS_LED_TIM_RCC              RCC_APB2Periph_TIM11

#define I2C_SCL_Pin                     GPIO_Pin_6
#define I2C_SCL_GPIO_Port               GPIOB

#define I2C_SDA_Pin                     GPIO_Pin_7
#define I2C_SDA_GPIO_Port               GPIOB

/* Initialize the system control */
void Sys_Ctrl_Init(void);
/* Switch the USB power to off, battery charging or heating */
void SetUSBPower(enum eUSBPower power_setting);
/* Switch capacitive sensing driven shield on or off */
void SetCapSenseShield(bool on);

#ifndef LED_PWM
/* Turn the LED on or off */
void SetLED(bool on);
#endif

#endif