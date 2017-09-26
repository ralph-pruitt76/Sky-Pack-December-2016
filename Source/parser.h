/**
  ******************************************************************************
  * File Name          : parser.h
  * Description        : This file provides code for the control of the baromoeter
  *                      hardware based on the LPS22HB chip.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __parser_H
#define __parser_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32l1xx_hal.h"
//#include "stm32l1xx_nucleo.h"
//#include "i2c.h"
#include <string.h>
#include "lsm6ds3.h"
#include "lis3mdl.h"
#include "lps25hb.h"
#include "pct2075.h"
#include "opt3001.h"
//#include "usart.h"
//#include "gpio.h"
//#include "barometer.h"
//#include "VMonitor.h"
//#include "GridEye.h"
//#include "Humidity.h"
//#include "Temperature.h"
//#include "RGBLight.h"
//#include "Rd_Sound.h"
//#include "main.h"
//#include "mbed.h"
#include "stm32l1xx.h"
#include "stdbool.h"
#include "bgm111.h"
#include "main.h"

/* Defines */
#define I2C_TIMEOUT     500            // Set as 500msec
#define SCALE_DELAY     10            // Scale Delay.
#define BUFFER_SIZE     40             // Maximum Buffer Size

/* Prototypes */
HAL_StatusTypeDef SkyBrd_ParserInit( void );
HAL_StatusTypeDef SkyBrd_ParserTsk(char *tempBffr);
HAL_StatusTypeDef SkyBrd_ProcessParserTsk( void );
HAL_StatusTypeDef SkyBrd_ParseString(char *tempBffr, bool BLE_Flag);
bool Tst_Bypass( void);
int isHexNum(char *ptr);
int hatoi( char *ptr );
//void sleep(void);
//void deepsleep(void);

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/