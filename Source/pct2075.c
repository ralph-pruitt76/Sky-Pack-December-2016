/* PCT2075 temperature sensor driver */

#include "pct2075.h"


/* Sensor Configuration */
/* Provide an init stucture, returns PCT2075_OK or PCT2075_ERR */

uint8_t PCT2075_Init(PCT2075_InitTypeDef *PCT2075_InitStruct)
{
  uint8_t cfg_reg = 0;
  int16_t temp_reg;
  
  /* Pointer check */
  if (!PCT2075_InitStruct) return PCT2075_ERR;
  
  /* Construct config register from fields in init structure */
  cfg_reg |= PCT2075_InitStruct->OS_Queue & PCT2075_OSFQUEUE_MASK;
  cfg_reg |= PCT2075_InitStruct->OS_Polarity & PCT2075_OSPOL_MASK;
  cfg_reg |= PCT2075_InitStruct->OS_Mode & PCT2075_OSMODE_MASK;
  cfg_reg |= PCT2075_InitStruct->Device_Mode & PCT2075_DEVMODE_MASK;
  /* Send the config register to the chip */
  PCT2075_WriteByte(PCT2075_CONF_ADDR, cfg_reg);
  
  /* Send overtemperature threshold and hysteresis to the chip */
  temp_reg = (int16_t)(PCT2075_InitStruct->OS_Trigger * 256.0f);
  PCT2075_WriteWord(PCT2075_TOS_ADDR, (uint16_t)temp_reg);
  temp_reg = (int16_t)(PCT2075_InitStruct->OS_Hyst * 256.0f);
  PCT2075_WriteWord(PCT2075_TOS_ADDR, (uint16_t)temp_reg);
  
  /* Send idle time to the chip (100ms increments) */
  PCT2075_WriteByte(PCT2075_IDLE_ADDR, PCT2075_InitStruct->Idle_Time);
  
  return PCT2075_OK;
}

/* Get the temperature */

float PCT2075_GetTemp(void)
{
  return (float)PCT2075_ReadWord(PCT2075_TEMP_ADDR) / 256.0f;
}
