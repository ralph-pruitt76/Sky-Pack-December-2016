/**
  ******************************************************************************
  * File Name          : ErrorCodes.h
  * Description        : This file provides code for the processing and control
  * error buffer.
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
#ifndef __ErrorCodes_H
#define __ErrorCodes_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <stdbool.h>
   
/* Defines */
#define ERR_DATA_LENGTH         100             // Number of codes to log.

typedef enum
{
  ERROR_NULL		= 0x0000,  		// NULL...NO Error Code
  ERROR_I2CBUSY         = 0x0001,               // I2C Bus detected busy. Rogue I2C part holding bus low.
  ERROR_BGMSYNC         = 0x0002,               // BGM111 processing code has detected a sync error on traffic from BGM111.
  ERROR_PRESSURE_INIT   = 0x0003,               // Initialization of Pressure Sensor failed.
  ERROR_BGMBUF_FULL     = 0x0004,               // BGM111 processing code has detected a Receive Buffer Full error on traffic from BGM111.
  ERROR_I2C_SCLK        = 0x0005,               // I2C Bus Test Failed. SCLK held low.
  ERROR_I2C_SDAT        = 0x0006,               // I2C Bus Test Failed. SDAT held low.
  ERROR_BGM_CNNCT       = 0x0007,               // BGM111 processing code has detected a Connection Dropped Event.
  ERROR_BGM_HRTBT       = 0x0008,               // BGM111 processing code has detected a Heart Beat Timeout Event.
  ERROR_MISC            = 0x0009,               // Misc Error that cannot be cartaloged.
  ERROR_IMU_INIT        = 0x000A,               // IMU Initialization has failed.
  ERROR_ILL_INIT        = 0x000B,               // Illuminance Initialization has failed.
  ERROR_IMU_ERR         = 0x000C,               // IMU Tasking has failed.
  ERROR_PRESSURE_ERR    = 0x000D,               // Pressure Sensor Tasking has failed.
  ERROR_BGM_OVERFLOW    = 0x000E,               // BGM111 processing code has detected Serial Stream Overflow.
  ERROR_TXBGMBUF_FULL   = 0x000F,               // BGM111 processing code has detected a Transmit Buffer Full error on traffic to BGM111.
  ERROR_FRAME_INIT      = 0x0010,               // Could not Initialize Flash Frame.
  ERROR_CAL_INIT        = 0x0011,               // Could not Initialize Calibration Flash Structure.
  ERROR_BGM_SYNCCNCT    = 0x0012,               // BGM111 processing code has detected a SYNC Connection Dropped Event.
  ERROR_APP_CNRYCNCT    = 0x0013,               // App Cade has detected a Canary Count Overflow. Timer Int has died.
  ERROR_ILL_ERR         = 0x0014,               // Illuminance Tasking has failed.

  ERROR_END             = 0xffff,       	// End Of Error Code List
}ErrorCodes;

typedef enum
{
  MODULE_NULL		= 0x0000,	// NULL...NO Device Code
  MODULE_main		= 0x0001,	// module: main.c
  MODULE_bgm111		= 0x0002,	// module: bgm111.c
  MODULE_i2c		= 0x0003,	// module: i2c_bus.c
  MODULE_Reset          = 0x0004,       // module: app_data.c: SkyPack_Reset( int code )
  MODULE_AppData        = 0x0005,       // module: app_data.c
  MODULE_MISC           = 0xffff,	// End Of Device Code List
}ModuleCodes;

// Structure
typedef struct ErrorElmnt {
   ErrorCodes 	ErrorCd;			// 16 Bit Code representing Error Code
   ModuleCodes 	DeviceCd;			// 16 Bit Code representing routine that detected Error.
    uint32_t	halTick;		        // HAL Tick Count when error was logged.......tickstart = HAL_GetTick();
 } ErrorElement;
typedef struct ErrorElmnt *ErrorElmntPtr;

/* Prototypes */
HAL_StatusTypeDef SkPck_ErrCdInit( void );
uint16_t  SkPck_ErrCdGetErrCnt( void );
ErrorElmntPtr  SkPck_ErrCdGetCrntErrCd( void );
ErrorElmntPtr  SkPck_ErrCdGetNxtErrCd( void );
HAL_StatusTypeDef SkPck_ErrCdLogErrCd( ErrorCodes ErrorCd, ModuleCodes DeviceCd );
ErrorCodes Get_ErrorCode( int code );

void HardFault_Handler(void);
void MemManage_Handler(void);
void UsageFault_Handler(void);
#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
