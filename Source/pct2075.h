/* PCT2075 temperature sensor header file */

#ifndef PCT2075_H_
#define PCT2075_H_

#include "i2c_bus.h"


/* Temperature sensor I2C address (8-bit) */

#define PCT2075_I2C_ADDRESS             0x92

/* Alias: access the PCT2075 by accessing the I2C bus */

#define PCT2075_ReadByte(reg)   \
        I2C_Read8bit(PCT2075_I2C_ADDRESS, (reg))
#define PCT2075_WriteByte(reg, value)  \
        I2C_Write8bit(PCT2075_I2C_ADDRESS, (reg), (value))
#define PCT2075_ReadWord(reg)   \
        I2C_Read16bitBE(PCT2075_I2C_ADDRESS, (reg))
#define PCT2075_WriteWord(reg, value)  \
        I2C_Write16bitBE(PCT2075_I2C_ADDRESS, (reg), (value))

/* PCT2075 init struct */

typedef struct
{
  uint8_t Device_Mode;
  uint8_t OS_Queue;
  uint8_t OS_Polarity;
  uint8_t OS_Mode;
  float OS_Trigger;
  float OS_Hyst;
  uint8_t Idle_Time;
} PCT2075_InitTypeDef;

/* OK and error definitions */

#define PCT2075_OK                      1
#define PCT2075_ERR                     0

/* PCT2075 register definitions */

#define PCT2075_TEMP_ADDR               0x00
#define PCT2075_CONF_ADDR               0x01
#define PCT2075_THYST_ADDR              0x02
#define PCT2075_TOS_ADDR                0x03
#define PCT2075_IDLE_ADDR               0x04

/* Configuration definitions */

/* OS fault queue field */

#define PCT2075_OSFQUEUE_1              0x00
#define PCT2075_OSFQUEUE_2              0x08
#define PCT2075_OSFQUEUE_4              0x10
#define PCT2075_OSFQUEUE_6              0x18

#define PCT2075_OSFQUEUE_MASK           0x18

/* OS polarity field */

#define PCT2075_OSPOL_LOW               0x00
#define PCT2075_OSPOL_HIGH              0x04

#define PCT2075_OSPOL_MASK              0x04

/* OS mode field */

#define PCT2075_OSMODE_COMP             0x00
#define PCT2075_OSMODE_INT              0x02

#define PCT2075_OSMODE_MASK             0x02

/* Device mode field */

#define PCT2075_DEVMODE_NORMAL          0x00
#define PCT2075_DEVMODE_SHDN            0x01

#define PCT2075_DEVMODE_MASK            0x01

/* Sensor Configuration */
/* Provide an init stucture, returns PCT2075_OK or PCT2075_ERR */

HAL_StatusTypeDef PCT2075_Init(PCT2075_InitTypeDef *PCT2075_InitStruct);

/* Data functions */

float PCT2075_GetTemp(void);


#endif