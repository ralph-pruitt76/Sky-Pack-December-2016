/**
  ******************************************************************************
  * File Name          : WWDG.h
  * Description        : This file provides code for the configuration
  *                      of the WWDG instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
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
#ifndef __wwdg_H
#define __wwdg_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_flash.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "main.h"
//#include "stm32l1xx_hal.h"
#include "stdbool.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

//extern WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN Private defines */

// Enums
typedef enum Code_ld
{
  NO_CODE       = 0, 
  SLOT0_CD      = 1,
  SLOT1_CD      = 2,
  SLOT2_CD      = 3,
} Code_load;
  /* WWDG clock counter = (PCLK1 (32MHz)/4096)/8) = 4 MHz (250ns) 
     WWDG Window value = 80 means that the WWDG counter should be refreshed only 
     when the counter is below 80 (and greater than 64/0x40) otherwise a reset will 
     be generated. 
     WWDG Counter value = 100, WWDG timeout = ~1024 us * 64 = 65.57 ms */
#define READ_REG(REG)         ((REG))

#define SKYBRD_TIMEOUT 127              // Set at 130.048 ms
                                        // Min_Data = 0x40 and Max_Data = 0x7F
                                        // This means the valid window is less than 80 and
                                        // greater than 64...OR
                                        // 65.536ms < WINDOW < 102.4ms
#define SKYBRD_HIGHLMIT 100             // Set at 102.4 ms
#define SKYBRD_LOWLMIT  64              // Set at 65.5 ms
                                         // Max_Data = 0x80 */
#define FRAME_SIZE       10              // Will Save 10 Frames
#define FRAME_CHKSUM     0x5a5a5a5a      // Code to determine if frame ha been Initialized
#ifdef STM32L151CBT6
  #define BASE_FLASH_ADDRESS      0x08018000        // Base Address to place all key Flash Structures.....32L151CBT6 Long..
#endif
#define BOOT_WAIT               80      // Default Wait time for Boot Sequence...80 Seconds.
#define DATE_STRING_LENGTH      30      // Length of Saved Date String.
#define VERSION_STRING_LENGTH   40      // Version String Length

// Private Structure
// wwdg Save Frame

typedef struct wwdg_SaveFrm
{
  bool  event;
} wwdg_SaveFrame;
typedef struct wwdg_SaveFrm *wwdg_SaveFrmPtr;

typedef struct wwdg_Frmes
{
  uint32_t checksum;
  // Version String
  char  Version_String[VERSION_STRING_LENGTH];  // Version String.
  // Boot Variables
  Code_load Load_Active;                // Code Load that is currently active.
  uint32_t      Boot_Addr[3];           // Boot Address Lookup Table.
  uint32_t      Boot_Vector[3];         // Boot Vector Lookup Table.
  uint32_t      Boot_Chksum[3];         // Boot Chksum Lookup Table.
  // Key Misc Variables
  uint32_t      SnsrTickCnt;
  uint32_t      TackLimit;
  uint32_t      BootDelay;
  bool          Units_flg;
  // Key Frame Tracking Variables
  uint8_t Frame_WrtPtr;
  uint8_t Frame_RdPtr;
  // Definition of Frames.
  wwdg_SaveFrame Saved_Frames[FRAME_SIZE];
} wwdg_Frames;
typedef struct wwdg_Frmes *wwdg_FrmesPtr;

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_WWDG_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef SkyBrd_WWDG_InitializeDateString( void );
char *SkyBrd_WWDG_GetDateString( void );
HAL_StatusTypeDef SkyBrd_WWDG_SetDateString( char* parmString );
HAL_StatusTypeDef SkyBrd_WWDG_InitializeTickString( void );
char *SkyBrd_WWDG_GetTickString( void );
HAL_StatusTypeDef RoadBrd_WWDG_SetTickString( char* parmString );
HAL_StatusTypeDef SkyBrd_WWDG_Start( void );
HAL_StatusTypeDef SkyBrd_WWDG_Refresh( void );
uint32_t SkyBrd_WWDG_GetRefreshCnt( void );
bool SkyBrd_WWDG_VerifyFrame( void );
#ifdef STM32L151CBT6
  HAL_StatusTypeDef SkyBrd_WWDG_InitializeFrmFlash( void );
  HAL_StatusTypeDef SkyBrd_WWDG_ReadFrmFlash( void );
  HAL_StatusTypeDef SkyBrd_WWDG_WriteFrmFlash( void );
  HAL_StatusTypeDef SkyBrd_WWDG_WriteFlash( wwdg_SaveFrame* Write_Frame );
  HAL_StatusTypeDef SkyBrd_WWDG_ReadFlash( wwdg_SaveFrame* Read_Frame );
  HAL_StatusTypeDef SkyBrd_Set_SnsrTickCnt( uint32_t PassedSnsrTickCnt );
  HAL_StatusTypeDef SkyBrd_Set_TackLimit( uint32_t PassedTackLimit );
  uint32_t SkyBrd_GetSampleTime( void );
  HAL_StatusTypeDef SkyBrd_Set_UnitsFlag( bool PassedUnitsFlag );
  uint32_t SkyBrd_Get_SnsrTickCnt( void );
  bool SkyBrd_Get_UnitsFlag( void );
  uint32_t SkyBrd_Get_TackLimit( void );
  HAL_StatusTypeDef SkyBrd_Set_BootDelay( uint32_t PassedBootDelay );
  uint32_t SkyBrd_Get_BootDelay( void );
  HAL_StatusTypeDef SkyBrd_WWDG_SetTickString( char* parmString );
  HAL_StatusTypeDef SkyBrd_Set_TmpSnsrTickCnt( uint32_t PassedSnsrTickCnt );
#endif
HAL_StatusTypeDef SkyBrd_Set_TickCounts( uint32_t PassedRdSndTickCnt, uint32_t PassedSnsrTickCnt );
char *SkyBrd_Get_VersionString( void );
HAL_StatusTypeDef SkyBrd_Set_VersionString( char *PassedVersionString );

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ wwdg_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
