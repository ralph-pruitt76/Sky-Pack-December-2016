/**
  ******************************************************************************
  * File Name          : WWDG.c
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

/* Includes ------------------------------------------------------------------*/
#include "wwdg.h"
//#include "gpio.h"
#include "usart.h"
#include "Flash.h"
//#include "tim.h"
#include "app_data.h"
#include <string.h>

/* USER CODE BEGIN 0 */
// Frame Structure Define
//wwdg_Frames wwdg_HardFrames  @ 0x08070000;
#ifdef STM32L151CBT6
  wwdg_Frames wwdg_HardFrames  @ BASE_FLASH_ADDRESS;

static wwdg_Frames Save_Frames;

static char DateString[DATE_STRING_LENGTH];             // Current Date String.
static char TickString[DATE_STRING_LENGTH];             // Current Tick String.

#endif

/* USER CODE END 0 */

//WWDG_HandleTypeDef hwwdg;

// wwdg Save Frame
static wwdg_SaveFrame wwdg_Save;

  /**
  * @brief  This function initializes the Static Current Date String.
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  */
HAL_StatusTypeDef SkyBrd_WWDG_InitializeDateString( void )
{
  strcpy(DateString, "---NULL---");
  return HAL_OK;
}

  /**
  * @brief  This function initializes the Static Current Date String.
  * @param  none
  * @retval char *:     Pointer to Date String.
  */
char *SkyBrd_WWDG_GetDateString( void )
{
  return &DateString[0];
}

  /**
  * @brief  This function initializes the Static Current Date String.
  * @param  char* parmString: String to be set.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  */
HAL_StatusTypeDef SkyBrd_WWDG_SetDateString( char* parmString )
{
  strcpy(DateString, parmString);
  return HAL_OK;
}

  /**
  * @brief  This function initializes the Static Current Date String.
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  */
HAL_StatusTypeDef SkyBrd_WWDG_InitializeTickString( void )
{
  strcpy(TickString, "---NULL---");
  return HAL_OK;
}

  /**
  * @brief  This function initializes the Static Current Date String.
  * @param  none
  * @retval char *:     Pointer to Date String.
  */
char *SkyBrd_WWDG_GetTickString( void )
{
  return &TickString[0];
}

  /**
  * @brief  This function initializes the Static Current Date String.
  * @param  char* parmString: String to be set.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  */
HAL_StatusTypeDef SkyBrd_WWDG_SetTickString( char* parmString )
{
  strcpy(TickString, parmString);
  return HAL_OK;
}

/* WWDG init function */
void MX_WWDG_Init(void)
{

  /*##-2- Configure the WWDG peripheral ######################################*/
  /* WWDG clock counter = (PCLK1 (32MHz)/4096)/8) = 976.6 Hz (1.02ms) 
     WWDG Window value = 80 means that the WWDG counter should be refreshed only 
     when the counter is below 80 (and greater than 64/0x40) otherwise a reset will 
     be generated. 
     WWDG Counter value = 127, WWDG timeout = ~1024 us * 64 = 65.57 ms */
  WWDG_SetPrescaler(WWDG_Prescaler_8);
  WWDG_SetWindowValue(SKYBRD_HIGHLMIT);		        // Set High end of 102.4 ms.
  WWDG_SetCounter(SKYBRD_TIMEOUT);			// Set Timer at 130.048 ms. 
}

void HAL_WWDG_MspInit( void )
{
  /* Peripheral clock enable */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_WWDG, ENABLE);
}

void HAL_WWDG_MspDeInit( void )
{
  WWDG_DeInit();
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  Start WWDG Timer
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Error Code logged
  *                                HAL_ERROR:    Error Log full or Bad Code
  */
HAL_StatusTypeDef SkyBrd_WWDG_Start( void )
{
  WWDG_EnableIT();
  return HAL_OK;
}

/**
  * @brief  Refresh WWDG Timer
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Error Code logged
  *                                HAL_ERROR:    Error Log full or Bad Code
  */
HAL_StatusTypeDef SkyBrd_WWDG_Refresh( void )
{
  uint32_t rfrsh_cnt;
  
  // Let's Only Refresh in Window...
  rfrsh_cnt = SkyBrd_WWDG_GetRefreshCnt();
  if ( rfrsh_cnt < SKYBRD_HIGHLMIT)
    WWDG_SetCounter(SKYBRD_TIMEOUT);

  return HAL_OK;
}

/**
  * @brief  Refresh WWDG Timer
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Error Code logged
  *                                HAL_ERROR:    Error Log full or Bad Code
  */
uint32_t SkyBrd_WWDG_GetRefreshCnt( void )
{
  // Return the 7-Bit Counter Value.
  return(READ_REG(WWDG->CR) & 0x7f);                 // hwwdg->Instance->CR
}

    
void HAL_WWDG_WakeupCallback( void )
{
  //uint8_t tempBffr2[20];

  static bool OnceFlg = false;
  
  if ( OnceFlg )
  {
    wwdg_Save.event = true;   // We have had a wwdg Event...Mark that it did occur.
//    if (wwdg_Save2.event)
//      OnceFlg = false;
  }
  else
  {
    OnceFlg = true;
  }
}

/**
* @brief This function handles wwdt global interrupt.
*/
void WWDG_IRQHandler(void)
{
  //HAL_WWDG_IRQHandler();
}

bool SkyBrd_WWDG_TstEvent( void )
{
  return wwdg_Save.event;
}

#ifdef STM32L151CBT6
  /**
  * @brief  This function verifies the WWDG Flash Frame Structure.
  * @param  none
  * @retval bool:     true:       Valid Frames
  *                   false:      Frame Bad.
  */
bool SkyBrd_WWDG_VerifyFrame( void )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  // Read Frame from Flash.
  Status = SkyBrd_FlashRead(  (uint32_t)&wwdg_HardFrames, (uint32_t *)&Save_Frames, sizeof(Save_Frames));
  // Compare SYnc Workd and return status.
  if (Status != HAL_OK)
    return false;
  else
  {
    if (Save_Frames.checksum == FRAME_CHKSUM)
      return true;
    else
      return false;
  }
}

  /**
  * @brief  This function initializes the key frame structures needed to track wwdg Frames.
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_WWDG_InitializeFrmFlash( void )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  // Initialize Key Structures of Frame
  Save_Frames.checksum = FRAME_CHKSUM;
  Save_Frames.SnsrTickCnt = PROCESS_SNSR_TIME;
  Save_Frames.TackLimit = TACK_LIMIT;
  Save_Frames.BootDelay = BOOT_WAIT;
  Save_Frames.Units_flg = true;
  
  Save_Frames.Frame_RdPtr = 0;
  Save_Frames.Frame_WrtPtr = 0;
  
  // Write Structure to Flash Memory.
  //Status = SkyBrd_FlashInitWrite( 0x00, 
  Status = SkyBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&wwdg_HardFrames, 
                               (uint32_t *)&Save_Frames, 
                               sizeof(Save_Frames));
  return Status;
}

/**
  * @brief  Update Key Tick Counts.
  * @param  uint32_t PassedRdSndTickCnt
  * @param  uint32_t PassedSnsrTickCnt
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_Set_TickCounts( uint32_t PassedRdSndTickCnt, uint32_t PassedSnsrTickCnt )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  Save_Frames.SnsrTickCnt = PassedSnsrTickCnt;
  ChangeSampleTimer();
  // Write Structure to Flash Memory.
  //Status = SkyBrd_FlashInitWrite( 0x00, 
  Status = SkyBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&wwdg_HardFrames, 
                               (uint32_t *)&Save_Frames, 
                               sizeof(Save_Frames));
  return Status;
}

/**
  * @brief  Update SnsrTickCnt.
  * @param  uint32_t PassedSnsrTickCnt
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_Set_SnsrTickCnt( uint32_t PassedSnsrTickCnt )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  Save_Frames.SnsrTickCnt = PassedSnsrTickCnt;
  ChangeSampleTimer();
  // Write Structure to Flash Memory.
  //Status = SkyBrd_FlashInitWrite( 0x00, 
  Status = SkyBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&wwdg_HardFrames, 
                               (uint32_t *)&Save_Frames, 
                               sizeof(Save_Frames));
  return Status;
}

HAL_StatusTypeDef SkyBrd_Set_TmpSnsrTickCnt( uint32_t PassedSnsrTickCnt )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  ChangeSampleTimer();
  return Status;
}

/**
  * @brief  Update Units_flg.
  * @param  bool PassedUnitsFlag
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_Set_UnitsFlag( bool PassedUnitsFlag )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  Save_Frames.Units_flg = PassedUnitsFlag;
  // Write Structure to Flash Memory.
  //Status = RoadBrd_FlashInitWrite( 0x00, 
  Status = SkyBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&wwdg_HardFrames, 
                               (uint32_t *)&Save_Frames, 
                               sizeof(Save_Frames));
  return Status;
}

/**
  * @brief  Update TackLimit.
  * @param  uint32_t PassedTackLimit
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_Set_TackLimit( uint32_t PassedTackLimit )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  Save_Frames.TackLimit = PassedTackLimit;
  // Write Structure to Flash Memory.
  //Status = RoadBrd_FlashInitWrite( 0x00, 
  Status = SkyBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&wwdg_HardFrames, 
                               (uint32_t *)&Save_Frames, 
                               sizeof(Save_Frames));
  return Status;
}

/**
  * @brief  Update BootDelay.
  * @param  uint32_t PassedBootDelay
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_Set_BootDelay( uint32_t PassedBootDelay )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  Save_Frames.BootDelay = PassedBootDelay;
  // Write Structure to Flash Memory.
  //Status = RoadBrd_FlashInitWrite( 0x00, 
  Status = SkyBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&wwdg_HardFrames, 
                               (uint32_t *)&Save_Frames, 
                               sizeof(Save_Frames));
  return Status;
}

/**
  * @brief  Retrieve SnsrTickCnt.
  * @param  None
  * @retval uint32_t Save_Frames.SnsrTickCnt
  */
uint32_t SkyBrd_Get_SnsrTickCnt( void )
{
  return Save_Frames.SnsrTickCnt;
}

 /**
  * @brief  Retrieve Units Flag.
  * @param  None
  * @retval bool Save_Frames.Units_flg
  */
bool SkyBrd_Get_UnitsFlag( void )
{
  return Save_Frames.Units_flg;
}

/**
  * @brief  Retrieve SnsrTickCnt.
  * @param  None
  * @retval uint32_t Save_Frames.SnsrTickCnt
  */
uint32_t SkyBrd_GetSampleTime( void )
{
  return Save_Frames.SnsrTickCnt;
}

/**
  * @brief  Retrieve TackLimit.
  * @param  None
  * @retval uint32_t Save_Frames.TackLimit
  */
uint32_t SkyBrd_Get_TackLimit( void )
{
  return Save_Frames.TackLimit;
}

/**
  * @brief  Retrieve BootDelay.
  * @param  None
  * @retval uint32_t Save_Frames.BootDelay
  */
uint32_t SkyBrd_Get_BootDelay( void )
{
  return Save_Frames.BootDelay;
}

/**
  * @brief  This function Reads the key frame Information from Flash..
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_WWDG_ReadFrmFlash( void )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  // Read Structure from Flash Memory.
  Status = SkyBrd_FlashRead(  (uint32_t)&wwdg_HardFrames, 
                               (uint32_t *)&Save_Frames, 
                               sizeof(Save_Frames));
  return Status;
}

  /**
  * @brief  This function writes the key frame Information to Flash..
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_WWDG_WriteFrmFlash( void )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  // Write Structure to Flash Memory.
  Status = SkyBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&wwdg_HardFrames, 
                               (uint32_t *)&Save_Frames, 
                               sizeof(Save_Frames));
  return Status;
}

/**
  * @brief  This function attempts to write the passed Flash frame to the Flash Memory and Save it.
  * @param  wwdg_SaveFrame* Write_Frame: WWDG Frame to be written to flash.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_WWDG_WriteFlash( wwdg_SaveFrame* Write_Frame )
{
  // Is Frame Initialized?
  if (!(SkyBrd_WWDG_VerifyFrame()))
    if (SkyBrd_WWDG_InitializeFrmFlash() != HAL_OK)
      return HAL_ERROR;
  // First, Test to see if there is any room in Current Frame Structure.
  if (Save_Frames.Frame_WrtPtr >= FRAME_SIZE)
    return HAL_ERROR;
  else
  {
    // OK, Increment Write Pointer and save data.
    Save_Frames.Frame_WrtPtr++;
    Save_Frames.Saved_Frames[Save_Frames.Frame_WrtPtr].event = Write_Frame->event;
    //Write Contents to Flash Memory.
    return(SkyBrd_WWDG_WriteFrmFlash());
  }
}

/**
  * @brief  This function attempts to read from the Flash Memory to the the passed Flash frame.
  * @param  wwdg_SaveFrame* Write_Frame: WWDG Frame to be written to flash.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_WWDG_ReadFlash( wwdg_SaveFrame* Read_Frame )
{
  // Is Frame Initialized?
  if (Save_Frames.checksum != FRAME_CHKSUM)
    // Read the Frame.
    SkyBrd_WWDG_ReadFrmFlash();
  // First, Test to see if there is any room in Current Frame Structure.
  if (Save_Frames.Frame_RdPtr >= FRAME_SIZE)
    return HAL_ERROR;
  else
  {
    // OK, Increment Read Pointer and Read data.
    Save_Frames.Frame_RdPtr++;
    Read_Frame->event = Save_Frames.Saved_Frames[Save_Frames.Frame_RdPtr].event;
  }
  return HAL_OK;
}
#endif
/* USER CODE END 1 */


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
