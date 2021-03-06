/**
 ******************************************************************************
 * @file    lis3mdl.c
 * @author  MEMS Application Team
 * @version V1.3.0
 * @date    28-May-2015
 * @brief   This file provides a set of functions needed to manage the lis3mdl.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "lis3mdl.h"
#include <math.h>

/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @addtogroup LIS3MDL
 * @{
 */

/** @defgroup LIS3MDL_Private_Functions LIS3MDL_Private_Functions
 * @{
 */

/**
 * @brief  Set LIS3MDL Initialization
 * @param  LIS3MDL_Init the configuration setting for the LIS3MDL
 * @retval HAL_OK in case of success, an error code otherwise
 */
HAL_StatusTypeDef LIS3MDL_Init(MAGNETO_InitTypeDef *LIS3MDL_Init)
{
  uint8_t tmp1 = 0x00;
  
  /* Configure the low level interface ---------------------------------------*/
  if(LIS3MDL_IO_Init() != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /****** Magnetic sensor *******/
  
  if(LIS3MDL_IO_Read(&tmp1, LIS3MDL_M_MEMS_ADDRESS, LIS3MDL_M_CTRL_REG3_M, 1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Conversion mode selection */
  tmp1 &= ~(LIS3MDL_M_MD_MASK);
  tmp1 |= LIS3MDL_Init->M_OperatingMode;
  
  if(LIS3MDL_IO_Write(&tmp1, LIS3MDL_M_MEMS_ADDRESS, LIS3MDL_M_CTRL_REG3_M, 1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  if(LIS3MDL_IO_Read(&tmp1, LIS3MDL_M_MEMS_ADDRESS, LIS3MDL_M_CTRL_REG1_M, 1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Output data rate selection */
  tmp1 &= ~(LIS3MDL_M_DO_MASK);
  tmp1 |= LIS3MDL_Init->M_OutputDataRate;
  
  /* X and Y axes Operative mode selection */
  tmp1 &= ~(LIS3MDL_M_OM_MASK);
  tmp1 |= LIS3MDL_Init->M_XYOperativeMode;
  
  if(LIS3MDL_IO_Write(&tmp1, LIS3MDL_M_MEMS_ADDRESS, LIS3MDL_M_CTRL_REG1_M, 1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  if(LIS3MDL_IO_Read(&tmp1, LIS3MDL_M_MEMS_ADDRESS, LIS3MDL_M_CTRL_REG2_M, 1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Full scale selection */
  tmp1 &= ~(LIS3MDL_M_FS_MASK);
  tmp1 |= LIS3MDL_Init->M_FullScale;
  
  if(LIS3MDL_IO_Write(&tmp1, LIS3MDL_M_MEMS_ADDRESS, LIS3MDL_M_CTRL_REG2_M, 1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Configure interrupt lines */
  //LIS3MDL_IO_ITConfig();
  
  return HAL_OK;
  
  /******************************/
}


/**
 * @brief  Read ID of LIS3MDL Magnetic sensor
 * @param  m_id the pointer where the ID of the device is stored
 * @retval HAL_OK in case of success, an error code otherwise
 */
HAL_StatusTypeDef LIS3MDL_Read_M_ID(uint8_t *m_id)
{
  if(!m_id)
  {
    return HAL_ERROR;
  }
  
  return LIS3MDL_IO_Read(m_id, LIS3MDL_M_MEMS_ADDRESS, LIS3MDL_M_WHO_AM_I_ADDR, 1);
}


/**
 * @brief  Read raw data from LIS3MDL Magnetic sensor output register
 * @param  pData the pointer where the magnetometer raw data are stored
 * @retval HAL_OK in case of success, an error code otherwise
 * Returned as units...Degrees per second....11/6/17 RP
 */ 
HAL_StatusTypeDef LIS3MDL_M_GetAxesRaw(int16_t *pData)
{
  uint8_t tempReg[2] = {0, 0};
  
  if(LIS3MDL_IO_Read(&tempReg[0], LIS3MDL_M_MEMS_ADDRESS, (LIS3MDL_M_OUT_X_L_M | LIS3MDL_I2C_MULTIPLEBYTE_CMD),
                     2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  pData[0] = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
  if(LIS3MDL_IO_Read(&tempReg[0], LIS3MDL_M_MEMS_ADDRESS, (LIS3MDL_M_OUT_Y_L_M | LIS3MDL_I2C_MULTIPLEBYTE_CMD),
                     2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  pData[1] = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
  if(LIS3MDL_IO_Read(&tempReg[0], LIS3MDL_M_MEMS_ADDRESS, (LIS3MDL_M_OUT_Z_L_M | LIS3MDL_I2C_MULTIPLEBYTE_CMD),
                     2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  pData[2] = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);
  
  return HAL_OK;
}


/**
 * @brief Read data from LIS3MDL Magnetic sensor and calculate Magnetic in mgauss
 * @param pData the pointer where the magnetometer data are stored
 * @retval HAL_OK in case of success, an error code otherwise
 */
HAL_StatusTypeDef LIS3MDL_M_GetAxes(int32_t *pData)
{
  uint8_t tempReg = 0x00;
  int16_t pDataRaw[3];
  float sensitivity = 0;
  
  if(LIS3MDL_M_GetAxesRaw(pDataRaw) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  if(LIS3MDL_IO_Read(&tempReg, LIS3MDL_M_MEMS_ADDRESS, LIS3MDL_M_CTRL_REG2_M, 1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  tempReg &= LIS3MDL_M_FS_MASK;
  
  switch(tempReg)
  {
    case LIS3MDL_M_FS_4:
      sensitivity = 0.14;
      break;
    case LIS3MDL_M_FS_8:
      sensitivity = 0.29;
      break;
    case LIS3MDL_M_FS_12:
      sensitivity = 0.43;
      break;
    case LIS3MDL_M_FS_16:
      sensitivity = 0.58;
      break;
  }
  
  pData[0] = (int32_t)(pDataRaw[0] * sensitivity);
  pData[1] = (int32_t)(pDataRaw[1] * sensitivity);
  pData[2] = (int32_t)(pDataRaw[2] * sensitivity);
  
  return HAL_OK;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
