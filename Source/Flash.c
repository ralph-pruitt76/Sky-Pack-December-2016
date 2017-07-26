/**
  ******************************************************************************
  * File Name          : Flash.c
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

/* Includes ------------------------------------------------------------------*/
#include "Flash.h"
#include <math.h>
#include "stdbool.h"

// Frame Structure Define
//FrameStructure Frame_Save  @ 0x08070000;

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Uncomment this line to Enable Write Protection */
//#define WRITE_PROTECTION_ENABLE

/* Uncomment this line to Disable Write Protection */
#define WRITE_PROTECTION_DISABLE

/* Check the status of the switches */
/* Enable by default the disable protection */
#if !defined(WRITE_PROTECTION_ENABLE)&&!defined(WRITE_PROTECTION_DISABLE)
#define WRITE_PROTECTION_DISABLE
#endif /* !WRITE_PROTECTION_ENABLE && !WRITE_PROTECTION_DISABLE */

/* Both switches cannot be enabled in the same time */
#if defined(WRITE_PROTECTION_ENABLE)&&defined(WRITE_PROTECTION_DISABLE)
#error "Switches WRITE_PROTECTION_ENABLE & WRITE_PROTECTION_DISABLE cannot be enabled in the time!"
#endif /* WRITE_PROTECTION_ENABLE && WRITE_PROTECTION_DISABLE */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Address = 0;
uint32_t PageError = 0;
uint32_t ProtectedPAGE = 0x0;
__IO TestStatus MemoryProgramStatus = PASSED;
/*Variable used for Erase procedure*/
//static FLASH_EraseInitTypeDef EraseInitStruct;
/*Variable used to handle the Options Bytes*/
//static FLASH_OBProgramInitTypeDef OptionsBytesStruct;

//  uint32_t NbPages;     /*!< NbPages: Number of pages to be erased.
//                             This parameter must be a value between 1 and (max number of pages - value of Initial page)*/


  /**
  * @brief  This function Initializes Option Bytes and writes the specified data to Target Flash memory.
  * @param  uint32_t  FlashProtect: Specifies the sector(s) which are write protected between Sector 0 to 31.
  *                                 This parameter can be a combination of @ref FLASHEx_Option_Bytes_Write_Protection1
  *                                   @defgroup FLASHEx_Option_Bytes_Write_Protection1 FLASHEx Option Bytes Write Protection1
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_FlashInitOption( uint32_t  FlashProtect)
{
  HAL_StatusTypeDef Status;
  FLASH_Status Status1;
  /*Variable used to handle the Options Bytes*/
//  static FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  Status = HAL_OK;

  //*
  //*
  //* INITITIALIZE KEY STRUCTURES BEFORE STARTING OPERATION.
  //*
  //*
  Status = HAL_OK;
  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  FLASH_OB_Unlock();

#ifdef WRITE_PROTECTION_DISABLE
  /* Restore write protected pages */
  Status1 = FLASH_OB_WRPConfig(FlashProtect, DISABLE);
  if(Status1 != FLASH_COMPLETE)
    return HAL_ERROR;
  else
    Status = HAL_OK;
  
  /* Generate System Reset to load the new option byte values ***************/
  FLASH_OB_Unlock();
  
#elif defined WRITE_PROTECTION_ENABLE
  /* Get current write protected pages and the new pages to be protected ******/
  ProtectedPAGE =  OptionsBytesStruct.WRPSector0To31 | FlashProtect; 

  /* Restore write protected pages */
  Status1 = FLASH_OB_WRPConfig(FlashProtect, ENABLE);
  if(Status1 != FLASH_COMPLETE)
    return HAL_ERROR;
  else
    Status = HAL_OK;

  /* Generate System Reset to load the new option byte values ***************/
  FLASH_OB_Unlock();
  
#endif /* WRITE_PROTECTION_DISABLE */
  /* Lock the Options Bytes *************************************************/
  FLASH_OB_Unlock();

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  FLASH_Lock();
  return Status;
}

/**
  * @brief  This function Initializes Option Bytes and writes the specified data to Target Flash memory.
  * @param  uint32_t  FlashProtect: Specifies the sector(s) which are write protected between Sector 0 to 31.
  *                                 This parameter can be a combination of @ref FLASHEx_Option_Bytes_Write_Protection1
  *                                   @defgroup FLASHEx_Option_Bytes_Write_Protection1 FLASHEx Option Bytes Write Protection1
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @param  uint32_t TypeErase:     Page Erase only.
  *                                 This parameter can be a value of @ref FLASHEx_Type_Erase
  *                                   @defgroup FLASHEx_Option_Type FLASHEx Option Type
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @param  uint32_t FlashAddress:  Initial FLASH address to be erased and written.
  *                                 This parameter must be a value belonging to FLASH Programm address (depending on the devices)
  * @param  uint32_t *ReadAddress:  Address of data to be written to flash.
  * @param  uint32_t Size:          Number of bytes/pages to be erased and written. Note that the Pages are an increment of 256 Bytes rounded up.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_FlashInitWrite( uint32_t  FlashProtect,
                                      uint32_t TypeErase,
                                      uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size)
{
  HAL_StatusTypeDef Status;
  FLASH_Status Status1;

  uint32_t Address;
  uint32_t EndAddress;
  uint32_t *DataPtr;
  int x;
  
  /*Variable used for Erase procedure*/
  //static FLASH_EraseInitTypeDef EraseInitStruct;

  Status = HAL_OK;

  //*
  //*
  //* INITITIALIZE KEY STRUCTURES BEFORE STARTING OPERATION.
  //*
  //*
  Status = HAL_OK;
  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  FLASH_OB_Unlock();

#ifdef WRITE_PROTECTION_DISABLE
  /* Restore write protected pages */
  Status1 = FLASH_OB_WRPConfig(FlashProtect, DISABLE);
  if(Status1 != FLASH_COMPLETE)
    return HAL_ERROR;
  else
    Status = HAL_OK;
  
  /* Generate System Reset to load the new option byte values ***************/
  FLASH_OB_Unlock();
  
#elif defined WRITE_PROTECTION_ENABLE
  /* Get current write protected pages and the new pages to be protected ******/
  ProtectedPAGE =  OptionsBytesStruct.WRPSector0To31 | FlashProtect; 

  /* Restore write protected pages */
  Status1 = FLASH_OB_WRPConfig(FlashProtect, ENABLE);
  if(Status1 != FLASH_COMPLETE)
    return HAL_ERROR;
  else
    Status = HAL_OK;

  /* Generate System Reset to load the new option byte values ***************/
  FLASH_OB_Unlock();
  
#endif /* WRITE_PROTECTION_DISABLE */
  /* Lock the Options Bytes *************************************************/
  FLASH_OB_Unlock();

  //*
  //*
  //* ERASE TARGET FLASH MEMORY.
  //*
  //*
  /* The selected pages are not write protected *******************************/
  if ((FLASH_OB_GetWRP() & FlashProtect) == 0x00)
  {
    /* Erase Key Pages.************************************************/
    Address = FlashAddress;
    for (x=0; x<(Size/FLASH_PAGE_SIZE); x++)
    {
      Status1 = FLASH_ErasePage( Address );
      if (Status1 != FLASH_COMPLETE)
      {
        /* Lock the Flash to disable the flash control register access (recommended
        to protect the FLASH memory against possible unwanted operation) *********/
        FLASH_Lock();
        return HAL_ERROR;
      }
      else
        Status = HAL_OK;
      Address += FLASH_PAGE_SIZE;
    }

    //*
    //*
    //* WRITE TO TARGET FLASH MEMORY.
    //*
    //*
    Address = FlashAddress;
    EndAddress = FlashAddress + Size;
    DataPtr = ReadAddress;
    while (Address < EndAddress)
    {
      Status1 = FLASH_FastProgramWord( Address, *DataPtr++ );
      if (Status1 != FLASH_COMPLETE)
      {
        /* Lock the Flash to disable the flash control register access (recommended
        to protect the FLASH memory against possible unwanted operation) *********/
        FLASH_Lock();
        return HAL_ERROR;
      }
      else
      {
        Status = HAL_OK;
        Address = Address + 4;
      }
    }

    //*
    //*
    //* VERIFY WRITE TO TARGET FLASH MEMORY.
    //*
    //*
    /* Check the correctness of written data */
    Address = FlashAddress;
    EndAddress = FlashAddress + Size;
    DataPtr = ReadAddress;

    while (Address < EndAddress)
    {
      if((*(__IO uint32_t*) Address) != *DataPtr++)
      {
        Status = HAL_ERROR;
        /* Lock the Flash to disable the flash control register access (recommended
          to protect the FLASH memory against possible unwanted operation) *********/
        FLASH_Lock();
        return Status;
      }
      Address += 4;
    }
  } //EndIf ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)
  else
  { 
    /* The desired pages are write protected */ 
    Status = HAL_ERROR;
  } //EndElse ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  FLASH_Lock();
  return Status;
}

/**
  * @brief  This function writes the specified data to Target Flash memory.
  * @param  uint32_t  FlashProtect: Specifies the sector(s) which are write protected between Sector 0 to 31.
  *                                 This parameter can be a combination of @ref FLASHEx_Option_Bytes_Write_Protection1
  *                                   @defgroup FLASHEx_Option_Bytes_Write_Protection1 FLASHEx Option Bytes Write Protection1
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @param  uint32_t TypeErase:     Page Erase only.
  *                                 This parameter can be a value of @ref FLASHEx_Type_Erase
  *                                   @defgroup FLASHEx_Option_Type FLASHEx Option Type
  *                                   Module stm32l1xx_hal_flash_ex.h
  * @param  uint32_t FlashAddress:  Initial FLASH address to be erased and written.
  *                                 This parameter must be a value belonging to FLASH Programm address (depending on the devices)
  * @param  uint32_t *ReadAddress:  Address of data to be written to flash.
  * @param  uint32_t Size:          Number of bytes/pages to be erased and written. Note that the Pages are an increment of 256 Bytes rounded up.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef SkyBrd_FlashWrite( uint32_t  FlashProtect,
                                      uint32_t TypeErase,
                                      uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size)
{
  HAL_StatusTypeDef Status;
  FLASH_Status Status1;
  uint32_t Address;
  uint32_t EndAddress;
  uint32_t *DataPtr;
//  int x;

  Status = HAL_OK;

  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_Unlock();

  // Clear any outstanding Flash States before starting...
  SET_BIT(FLASH->SR, FLASH_SR_WRPERR);
  //*
  //*
  //* ERASE TARGET FLASH MEMORY.
  //*
  //*
  /* The selected pages are not write protected *******************************/
  if ((FLASH_OB_GetWRP() & FlashProtect) == 0x00)
  {
    /* Erase Key Pages.************************************************/
    Address = FlashAddress;
    // Loop Until all targeted memory has been erased.
    while( Address<(Size+FlashAddress))
    //for (x=0; x<=(Size/FLASH_PAGE_SIZE); x++)
    {
      Status1 = FLASH_ErasePage( Address );
      if (Status1 != FLASH_COMPLETE)
      {
        /* Lock the Flash to disable the flash control register access (recommended
        to protect the FLASH memory against possible unwanted operation) *********/
        FLASH_Lock();
        return HAL_ERROR;
      }
      else
        Status = HAL_OK;
      Address += FLASH_PAGE_SIZE;
    }

    //*
    //*
    //* WRITE TO TARGET FLASH MEMORY.
    //*
    //*
    Address = FlashAddress;
    EndAddress = FlashAddress + Size;
    DataPtr = ReadAddress;
    while (Address < EndAddress)
    {
      Status1 = FLASH_FastProgramWord( Address, *DataPtr++ );
      if (Status1 != FLASH_COMPLETE)
      {
        /* Lock the Flash to disable the flash control register access (recommended
        to protect the FLASH memory against possible unwanted operation) *********/
        FLASH_Lock();
        return HAL_ERROR;
      }
      else
      {
        Status = HAL_OK;
        Address = Address + 4;
      }
    }

    //*
    //*
    //* VERIFY WRITE TO TARGET FLASH MEMORY.
    //*
    //*
    /* Check the correctness of written data */
    Address = FlashAddress;
    EndAddress = FlashAddress + Size;
    DataPtr = ReadAddress;

    while (Address < EndAddress)
    {
      if((*(__IO uint32_t*) Address) != *DataPtr++)
      {
        Status = HAL_ERROR;
        /* Lock the Flash to disable the flash control register access (recommended
          to protect the FLASH memory against possible unwanted operation) *********/
        FLASH_Lock();
        return Status;
      }
      Address += 4;
    }
  } //EndIf ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)
  else
  { 
    /* The desired pages are write protected */ 
    Status = HAL_ERROR;
  } //EndElse ((OptionsBytesStruct.WRPSector0To31 & FlashProtect) == 0x00)

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  FLASH_Lock();
  return Status;
}

/**
  * @brief  This function reads the specified data from Target Flash memory.
  * @param  uint32_t FlashAddress:  Initial FLASH address to be read.
  * @param  uint32_t *ReadAddress:  Address of data to be read from flash.
  * @param  uint32_t Size:          Number of bytes to read.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  */
HAL_StatusTypeDef SkyBrd_FlashRead(  uint32_t FlashAddress,
                                      uint32_t *ReadAddress,
                                      uint32_t Size)
{
  uint32_t Address;
  uint32_t EndAddress;
  uint32_t *DataPtr;

  //* Read Data to Target Locations.
  Address = FlashAddress;
  EndAddress = FlashAddress + Size;
  DataPtr = ReadAddress;

  while (Address < EndAddress)
  {
    *DataPtr++ = (*(__IO uint32_t*) Address);
    Address += 4;
  }

  return HAL_OK;
}


/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
