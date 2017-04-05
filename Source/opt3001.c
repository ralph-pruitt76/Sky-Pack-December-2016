/* TI OPT3001 irradiance sensor driver */

#include "opt3001.h"
#include "usart.h"


/* Convert 32-bit to 16-bit fractional / exp value, preserving as much
 * precision as possible */

uint16_t toFracExp(uint32_t val)
{
  int exp = 0;
  while (val > 0x00000FFF)
  {
    val >>= 1;
    exp++;
  }
  return (uint16_t)(val | (exp << OPT3001_RESULT_EXP_SHIFT));
}

/* Convert 16-bit fractional / exp value to 32-bit */

uint32_t fromFracExp(uint16_t val)
{
  return (uint32_t)(val & OPT3001_RESULT_FRAC_MASK) <<
          (val >> OPT3001_RESULT_EXP_SHIFT);
}

/* Sensor Configuration */
/* Provide an init stucture, returns OPT3001_OK or OPT3001_ERR */

HAL_StatusTypeDef OPT3001_Init(OPT3001_InitTypeDef *OPT3001_InitStruct)
{
  uint16_t cfg_reg = 0;
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK; 
  
  /* Pointer check */
  if (!OPT3001_InitStruct) return HAL_ERROR;
  
  /* Check that we are talking to the correct chip */
  if (OPT3001_ReadReg(OPT3001_MFGID_ADDR) != 0x5449 ||
      OPT3001_ReadReg(OPT3001_DEVID_ADDR) != 0x3001) return HAL_ERROR;
  
  /* Construct config register from fields in init structure */
  cfg_reg |= ((uint16_t)OPT3001_InitStruct->Range << OPT3001_CONFIG_RN_SHIFT)
              & OPT3001_CONFIG_RN_MASK;
  cfg_reg |= OPT3001_InitStruct->Conversion_Time & OPT3001_CONFIG_CT_MASK;
  cfg_reg |= OPT3001_InitStruct->Mode & OPT3001_CONFIG_M_MASK;
  cfg_reg |= OPT3001_InitStruct->Int_Latch & OPT3001_CONFIG_L_MASK;
  cfg_reg |= OPT3001_InitStruct->Int_Pol & OPT3001_CONFIG_POL_MASK;
  cfg_reg |= OPT3001_InitStruct->Fault_Count & OPT3001_CONFIG_FC_MASK;
  /* Send the config register to the chip */
  Status = OPT3001_WriteReg(OPT3001_CONFIG_ADDR, cfg_reg);
  if (Status != HAL_OK)
    return Status;
  
  /* Send the low and high limit configuration to the chip */
  Status = OPT3001_WriteReg(OPT3001_LOW_LIMIT_ADDR,
                toFracExp(OPT3001_InitStruct->Low_Limit));
  if (Status != HAL_OK)
    return Status;
  Status = OPT3001_WriteReg(OPT3001_HIGH_LIMIT_ADDR,
                toFracExp(OPT3001_InitStruct->High_Limit));

  return Status;
}

/* Data functions */

uint32_t OPT3001_GetData(void)
{
  return fromFracExp(OPT3001_ReadReg(OPT3001_RESULT_ADDR));
}

/* Status functions */

uint16_t OPT3001_GetStatus(void)
{
  return OPT3001_ReadReg(OPT3001_CONFIG_ADDR);
}
