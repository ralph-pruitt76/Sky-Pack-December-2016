/**
  ******************************************************************************
  * File Name          : ErrorCodes.c
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

/* Includes ------------------------------------------------------------------*/
#include "ErrCodes.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>


// Constant Strings Definition
const char * const CodesArray[] = { "       ",                      // CODE 000: NULL...NO Error Code
                                    "I2CBUSY",                      // CODE 001: I2C Bus detected busy. Rogue I2C part holding bus low.
                                    "VM_INIT",                      // CODE 002: Initialization of V Monitor Code failed.
                                    "BGMSYNC",                      // CODE 003: BGM111 processing code has detected a sync error on traffic from BGM111.
                                    "GEYE_IN",                      // CODE 004: Initialization of Grid Eye Sensor failed.
                                    "PRSS_IN",                      // CODE 005: Initialization of Pressure Sensor failed.
                                    "BGMRFUL",                      // CODE 006: BGM111 processing code has detected a Receive Buffer Full error on traffic from BGM111.
                                    "HUMD_IN",                      // CODE 007: Initialization of Humidity Sensor failed.
                                    "RGBINIT",                      // CODE 008: Initialization of RGB Sensor failed.
                                    "TEMPINI",                      // CODE 009: Initialization of Temperature Sensor failed.
                                    "CEYE_IN",                      // CODE 010: Initialization of Cool Eye Sensor failed.
                                    "I2CSCLK",                      // CODE 011: I2C Bus Test Failed. SCLK held low.
                                    "I2CSDAT",                      // CODE 012: I2C Bus Test Failed. SDAT held low.
                                    "BGMCNCT",                      // CODE 013: BGM111 processing code has detected a Connection Dropped Event.
                                    "BGMHRBT",                      // CODE 014: BGM111 processing code has detected a Heart Beat Timeout Event.
                                    "ERRMISC",                      // CODE 015: Misc Error that cannot be cartaloged
                                    "       ",                      // CODE 016: NULL...
                                    "       " };                    // CODE 017: NULL...
const char * const ModuleArray[] = { "       ",                    // CODE 000: NULL...NO Device Code
                                     "   main",                    // CODE 001: module: main.c
                                     " bgm111",                    // CODE 002: module: bgm111.c
                                     "    i2c",                    // CODE 003: module: i2c.c...
                                     " SP_Rst",                    // CODE 004: module: app_data.c: SkyPack_Reset( int code )
                                     "       ",                    // CODE 005: module: NULL...
                                     "       ",                    // CODE 006: module: NULL...
                                     "       " };                  // CODE 007: module: NULL...
    
/* Error Code structure */
struct
{
  ErrorElement error_buf[ERR_DATA_LENGTH];
  volatile uint16_t error_wr;
  uint16_t error_rd;
} static errStruct;

//*
//* Define Internal Buffer handling routines.
//* 

void RBBuffer_LowLevel_Init(void)
{
  errStruct.error_wr = 0;
  errStruct.error_rd = 0;
}

/* Next buffer index based on current index and buffer size */
//#pragma inline=forced
uint16_t RBNextBufIdx(uint16_t idx)
{
  idx++;
  return idx < ERR_DATA_LENGTH ? idx : 0;
}

/* Report if the buffer is full based on its indexes */
//#pragma inline=forced
bool RBIsBufFull(uint16_t wr_idx, uint16_t rd_idx)
{
  return RBNextBufIdx(wr_idx) == rd_idx;
}

/* Get the used space in the buffer based on its indexes */
//#pragma inline=forced
uint16_t RBBufUsed(uint16_t wr_idx, uint16_t rd_idx)
{
  int size = (int)wr_idx - (int)rd_idx;
  if (size < 0)
  {
    size = ERR_DATA_LENGTH + size;
  }
  return size;
}

/* Get the free space in the buffer based on its indexes */
//#pragma inline=forced
uint16_t RBBufFree(uint16_t wr_idx, uint16_t rd_idx)
{
  return (ERR_DATA_LENGTH - 1) - RBBufUsed(wr_idx, rd_idx);
}

/**
  * @brief  This function initializes the Error Code Tracking Structure.
  * @retval HAL_StatusTypeDef:     HAL_OK:       No Errors
  *                                HAL_ERROR:    Error Found during initialization.
  */
HAL_StatusTypeDef SkPck_ErrCdInit( void )
{
  RBBuffer_LowLevel_Init();              // Reset all Internal Pointers.
  return HAL_OK;
}

/**
  * @brief  This function returns the current Error Count in the Error Buffer..
  * @retval uint16_t:     Number of errors being tracked
  */
uint16_t  SkPck_ErrCdGetErrCnt( void )
{
  return RBBufUsed(errStruct.error_wr, errStruct.error_rd);
}

 /**
  * @brief  This function returns the current pointer at the next code to be processed.
  * @retval ErrorElmntPtr:     Points to the next code to be processed.
  */
ErrorElmntPtr  SkPck_ErrCdGetCrntErrCd( void )
{
  return &errStruct.error_buf[errStruct.error_rd];
}

 /**
  * @brief  This function returns the current pointer at the next code to be processed and
  * 		   and increments the pointer to the next code.
  * @retval ErrorElmntPtr:     Points to the next code to be processed.
  */
ErrorElmntPtr  SkPck_ErrCdGetNxtErrCd( void )
{
  ErrorElmntPtr TempPtr;
  
  // Get Current Pointer
  TempPtr = &errStruct.error_buf[errStruct.error_rd];
  // Update Read Pointer and return.
  errStruct.error_rd = RBNextBufIdx(errStruct.error_rd);
  return TempPtr;
}

ErrorCodes Get_ErrorCode( int code )
{
  switch (code)
  {
    // Fatal Sync Error Detected.
    case FATAL_SYNC:
      return ERROR_BGMSYNC;
    // Fatal: payload bigger than 60 bytes, something's wrong.
    case FATAL_PAYLDSYNC:
      return ERROR_BGMSYNC;
    // Fatal Buffer Overflow.
    case FATAL_OVERFLOW:
      return ERROR_BGMBUF_FULL;
    // 90 Second Timeout.
    case FATAL_TIMEOUT:
      return ERROR_BGM_HRTBT;
    // Connection Dropped
    case FATAL_CNCTDROP:
      return ERROR_BGM_CNNCT;
    // I2C Channel Hung.
    case FATAL_I2CDROP:
      return ERROR_I2CBUSY;
    // Generic Fatal Error(USART).
    case FATAL_ERROR:
      return ERROR_MISC;
    default:
      return ERROR_MISC;
  }
}

/**
  * @brief  This function returns the current pointer at the next code to be processed and
  * 		   and increments the pointer to the next code.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Error Code logged
  *                                HAL_ERROR:    Error Log full or Bad Code
  */
HAL_StatusTypeDef SkPck_ErrCdLogErrCd( ErrorCodes ErrorCd, ModuleCodes DeviceCd )
{
  uint8_t tempBffr2[80];
  HAL_StatusTypeDef Status;
  
  // Is Buffer Full?
  if ( RBIsBufFull(errStruct.error_wr, errStruct.error_rd) )
    return HAL_ERROR;
  
  // Pass passed Information to indexed Entry.
  errStruct.error_buf[errStruct.error_wr].DeviceCd = DeviceCd;
  errStruct.error_buf[errStruct.error_wr].ErrorCd = ErrorCd;
  //errStruct.error_buf[errStruct.error_wr].halTick = HAL_GetTick();
  errStruct.error_buf[errStruct.error_wr].halTick = 0;
  
  // Update Write Pointer.
  errStruct.error_wr = RBNextBufIdx(errStruct.error_wr);
  //strcpy( (char *)tempBffr2, "ERROR: ERROR_I2CBUSY\r\n\r\n");
  //strcpy( (char *)tempBffr2, CodesArray[ErrorCd]);
  sprintf( (char *)tempBffr2, "%s ERROR: %s\r\n\r\n", ModuleArray[DeviceCd], CodesArray[ErrorCd]);
  // Send string to UART..
  Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2);
  
  return Status;
}


/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
