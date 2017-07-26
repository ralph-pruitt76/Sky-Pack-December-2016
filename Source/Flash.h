/**
  ******************************************************************************
  * File Name          : Flash.h
  * Description        : This file provides code for the control, reading and 
  *                      writing of the Flash Memory on the Design.
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
#ifndef __Flash_H
#define __Flash_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32l1xx_hal.h"
#include "wwdg.h"
//#include "Calibration.h"
#include "stm32l1xx_flash.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "main.h"
//#include "stdint.h"

/* Defines */
#define FLASH_PAGE_SIZE           ((uint32_t)256)

/** @defgroup FLASHEx_Type_Erase FLASHEx_Type_Erase
  * @{
  */
#define FLASH_TYPEERASE_PAGES           ((uint32_t)0x00)  /*!<Page erase only*/

// Structure
typedef struct FrameStruct {
  uint16_t temp1;
//  Calibration_Frames   Calibration_Frame;
//  wwdg_Frames          wwdg_Save;
} FrameStructure;
typedef struct FrameStruct *FrameStructPtr;
  

/* Prototypes */
HAL_StatusTypeDef SkyBrd_FlashInitOption( uint32_t  FlashProtect);
HAL_StatusTypeDef SkyBrd_FlashInitWrite( uint32_t  FlashProtect,
                                      uint32_t TypeErase,
                                      uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size);
HAL_StatusTypeDef SkyBrd_FlashWrite( uint32_t  FlashProtect,
                                      uint32_t TypeErase,
                                      uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size);
HAL_StatusTypeDef SkyBrd_FlashRead(  uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size);

#ifdef __cplusplus
}
#endif
#endif /*__Flash_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
