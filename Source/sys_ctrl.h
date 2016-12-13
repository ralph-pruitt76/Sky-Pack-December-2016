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

#define SHIELD_ON_GPIO_PORT             GPIOA
#define SHIELD_ON_PIN                   GPIO_Pin_5
#define SHIELD_ON_GPIO_CLK              RCC_AHBPeriph_GPIOA

#define CHARGE_ON_GPIO_PORT             GPIOA
#define CHARGE_ON_PIN                   GPIO_Pin_7
#define CHARGE_ON_GPIO_CLK              RCC_AHBPeriph_GPIOA

#define HEAT_ON_GPIO_PORT               GPIOA
#define HEAT_ON_PIN                     GPIO_Pin_8
#define HEAT_ON_GPIO_CLK                RCC_AHBPeriph_GPIOA

#define STATUS_LED_GPIO_PORT            GPIOB
#define STATUS_LED_PIN                  GPIO_Pin_9
#define STATUS_LED_GPIO_CLK             RCC_AHBPeriph_GPIOB
#define STATUS_LED_SOURCE               GPIO_PinSource9
#define STATUS_LED_AF                   GPIO_AF_TIM11
#define STATUS_LED_TIM                  TIM11
#define STATUS_LED_TIM_RCC              RCC_APB2Periph_TIM11


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