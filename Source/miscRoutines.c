/**
  ******************************************************************************
  * File Name          : miscRoutines.c
  * Description        : This file provides code for the processing and control
  * error buffer.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 WeatherCloud
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

/* Includes ------------------------------------------------------------------*/
#include "miscRoutines.h"
#include "sys_ctrl.h"
//#include "usart.h"
//#include <stdio.h>
//#include <string.h>


// Constant Strings Definition
GPIO_TypeDef* SkyPack_LED_PORT[SkyPack_LEDn] = {STATUS_LED_GPIO_PORT, 
                                                BLUE_GPIO_PORT, 
                                                GREEN_GPIO_PORT, 
                                                YELLOW_GPIO_PORT,
                                                VDD_ON_GPIO_PORT,
                                                BGM_ON_GPIO_PORT,
                                                RESET_BGM111_GPIO_PORT,
                                                CHARGE_ON_GPIO_PORT,
                                                HEAT_ON_GPIO_PORT};

const uint16_t SkyPack_LED_PIN[SkyPack_LEDn] = {STATUS_LED_PIN, 
                                                BLUE_PIN, 
                                                GREEN_PIN, 
                                                YELLOW_PIN,
                                                VDD_ON_PIN,
                                                BGM_ON_PIN,
                                                RESET_BGM111_PIN,
                                                CHARGE_ON_PIN,
                                                HEAT_ON_PIN};

    
/**
  * @brief  Turns selected gpio On.
  * @param  Led: Specifies the gpio to be set on. 
  *   This parameter can be one of following parameters:
  * @retval None
  */
void SkyPack_gpio_On(SkyPack_Led_TypeDef Port)
{
  GPIO_SetBits(SkyPack_LED_PORT[Port], SkyPack_LED_PIN[Port]);
}

/**
  * @brief  Turns selected gpio Off.
  * @param  Led: Specifies the gpio to be set off. 
  *   This parameter can be one of following parameters:
  * @retval None
  */
void SkyPack_gpio_Off(SkyPack_Led_TypeDef Port)
{
  GPIO_ResetBits(SkyPack_LED_PORT[Port], SkyPack_LED_PIN[Port]);
}


/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/

