/**
  ******************************************************************************
  * File Name          : miscRoutines.h
  * Description        : This file provides misc routines used for processing in
  *                      the Sky ack design
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
  *   3. Neither the name of WeatherCloud nor the names of its contributors
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
#ifndef __miscRoutines_H
#define __miscRoutines_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include <stdint.h>
#include <stdbool.h>
   
/* Defines */
typedef enum 
{
  LED1 = 0,
  BLUE_LED = 1,
  GREEN_LED = 2,
  YELLOW_LED = 3,
  gVDD_PWR = 4,
  gBGM_PWR = 5,
  gRESET_BGM111 = 6,
  gCHARGE_ON = 7,
  gHEAT_ON = 8,
  gI2C_CLK = 9,

  BGM_LED = BLUE_LED,
  MICRO_LED = GREEN_LED,
  STATUS_LED = YELLOW_LED,
  NUCLEO_LED_GREEN = LED1
} SkyPack_Led_TypeDef;
#define SkyPack_LEDn                     10

typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

// Structure

/* Prototypes */
void SkyPack_gpio_On(SkyPack_Led_TypeDef Port);
void SkyPack_gpio_Off(SkyPack_Led_TypeDef Port);
void SkyPack_gpio_TriState(SkyPack_Led_TypeDef Port);
void SkyPack_gpio_ReInit(SkyPack_Led_TypeDef Port);
void SkyPack_RePowerUpIO( void );
void SkyPack_PowerDnIO( void );
void SkyPack_LEDTest( void );
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
