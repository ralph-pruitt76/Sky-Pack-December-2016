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
#include "app_data.h"
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

/**
  * @brief  Turns selected gpio Input(Tri-State).
  * @param  Led: Specifies the gpio to be set Tri-State. 
  *   This parameter can be one of following parameters:
  * @retval None
  */
void SkyPack_gpio_TriState(SkyPack_Led_TypeDef Port)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SkyPack_LED_PIN[Port];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;                  // Set as Input....No Drive.
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                // Open Drain...NO Drive.
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;              // NO PULL.
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;              // Low Speed
  GPIO_Init(SkyPack_LED_PORT[Port], &GPIO_InitStructure);
}

/**
  * @brief  Re-Init selected gpio Input.
  * @param  Led: Specifies the gpio to be set initialized. 
  *   This parameter can be one of following parameters:
  * @retval None
  */
void SkyPack_gpio_ReInit(SkyPack_Led_TypeDef Port)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SkyPack_LED_PIN[Port];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  // Set as Output
  if (Port == gCHARGE_ON)
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;              // Open Drain...NO Drive.
  else
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;              // Normal Push-Pull.
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;              // NO PULL.
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;              // Low Speed
  GPIO_Init(SkyPack_LED_PORT[Port], &GPIO_InitStructure);
}

/**
  * @brief  Re-Power Up IO Pins to known state.
  * @param  None
  * @retval None
  */
void SkyPack_RePowerUpIO( void )
{
  // 1. Enable gRESET_BGM111 Pin and assert Low.
  SkyPack_gpio_ReInit(gRESET_BGM111);
  SkyPack_gpio_Off(gRESET_BGM111);
  // 2. Enable LEDs.
  SkyPack_gpio_ReInit(BGM_LED);
  SkyPack_gpio_Off(BGM_LED);
  SkyPack_gpio_ReInit(MICRO_LED);
  SkyPack_gpio_Off(MICRO_LED);
  SkyPack_gpio_ReInit(STATUS_LED);
  SkyPack_gpio_Off(STATUS_LED);
  // 3. Power up VDD
  SkyPack_gpio_On(gVDD_PWR);            // Turn on VDD(I2C Sensors).
  // 4. Wait 100msec for power plane o stabilize.
  delay_100msec(1);
  // 5. Power Up BGM111.
  SkyPack_gpio_On(gBGM_PWR);            // Turn on V+(BGM Power).
  // 6. Wait for power to stabilize...200msec
  delay_100msec(1);
}

/**
  * @brief  Power Down IO Pinsknown state.
  * @param  None
  * @retval None
  */
void SkyPack_PowerDnIO( void )
{
  // Power Down Planes in Order.
  //    1. Assert Reset Pin on BGM111
  SkyPack_gpio_Off(gRESET_BGM111);
  //    2. Wait 100 msec for it to stabilize.
  delay_100msec(1);
  //    3. Power down BGM111.
  SkyPack_gpio_TriState(gBGM_PWR);
  //    4. Wait 100 msec.
  delay_100msec(1);
  //    5. Power Down I2C Bus.
  SkyPack_gpio_TriState(gVDD_PWR);
  //    6. Wait 100 msec.
  delay_100msec(1);
  
  // NEXT, Lets set all IO Pins into correct state.
  SkyPack_gpio_TriState(gCHARGE_ON);
  SkyPack_gpio_TriState(gHEAT_ON);
  SkyPack_gpio_TriState(STATUS_LED);
  SkyPack_gpio_TriState(BGM_LED);
  SkyPack_gpio_TriState(MICRO_LED);
  SkyPack_gpio_TriState(gRESET_BGM111);
}

/**
  * @brief  Test LEDs by flashing each for 100msec.
  * @param  None
  * @retval None
  */
void SkyPack_LEDTest( void )
{
  //    1. Test Blue LED
  SkyPack_gpio_On(BLUE_LED);
  delay_100msec(1);
  SkyPack_gpio_Off(BLUE_LED);
  delay_100msec(1);
  //    2. Test Green LED
  SkyPack_gpio_On(GREEN_LED);
  delay_100msec(1);
  SkyPack_gpio_Off(GREEN_LED);
  delay_100msec(1);
  //    3. Test Yellow LED
  SkyPack_gpio_On(YELLOW_LED);
  delay_100msec(1);
  SkyPack_gpio_Off(YELLOW_LED);
  delay_100msec(1);
}

/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral for STM32L1XX family devices 
  * @param  GPIO_Pin: specifies the port bit to read.
  *         This parameter can be GPIO_PIN_x where x can be (0..15).
  * @retval The input port pin value.
  */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState bitstatus;

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = GPIO_PIN_SET;
  }
  else
  {
    bitstatus = GPIO_PIN_RESET;
  }
  return bitstatus;
}


/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/

