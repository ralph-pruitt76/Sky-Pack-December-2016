/* TI OPT3001 irradiance sensor header file
 * All lux levels are converted from the internal 16-bit fractional / exponent
 * format to 32-bit 1/100 lux values */

#ifndef OPT3001_H_
#define OPT3001_H_

#include "i2c_bus.h"


/* Irradiance sensor I2C address (8-bit) */

#define OPT3001_I2C_ADDRESS             0x88

/* Alias: access the OPT3001 by accessing the I2C bus */

#define OPT3001_ReadReg(reg)   \
        I2C_Read16bitBE(OPT3001_I2C_ADDRESS, (reg))
#define OPT3001_WriteReg(reg, data)  \
        I2C_Write16bitBE(OPT3001_I2C_ADDRESS, (reg), (data))

/* OPT3001 init struct */

typedef struct
{
  uint8_t  Range;                       /* Full scale range */
  uint16_t Conversion_Time;             /* Conversion time 100 ms or 800 ms */
  uint16_t Mode;                        /* Mode of operation */
  uint8_t  Int_Latch;                   /* Latch interrupt or transparent */
  uint8_t  Int_Pol;                     /* Interrupt polarity */
  uint8_t  Fault_Count;                 /* Fault count before int or flag */
  uint32_t Low_Limit;                   /* Low limit for fault detect */
  uint32_t High_Limit;                  /* High limit for fault detect */
} OPT3001_InitTypeDef;

/* OK and error definitions */

#define OPT3001_OK                      1
#define OPT3001_ERR                     0

/* OPT3001 register definitions */

#define OPT3001_RESULT_ADDR             0x00
#define OPT3001_CONFIG_ADDR             0x01
#define OPT3001_LOW_LIMIT_ADDR          0x02
#define OPT3001_HIGH_LIMIT_ADDR         0x03
#define OPT3001_MFGID_ADDR              0x7E
#define OPT3001_DEVID_ADDR              0x7F

/* Result register definitions */

#define OPT3001_RESULT_FRAC_MASK        0x0FFF
#define OPT3001_RESULT_EXP_MASK         0xF000
#define OPT3001_RESULT_EXP_SHIFT        12

/* Configuration definitions */

/* Range Number field */

#define OPT3001_CONFIG_RN_41LUX         0x00
#define OPT3001_CONFIG_RN_82LUX         0x01
#define OPT3001_CONFIG_RN_164LUX        0x02
#define OPT3001_CONFIG_RN_328LUX        0x03
#define OPT3001_CONFIG_RN_655LUX        0x04
#define OPT3001_CONFIG_RN_1310LUX       0x05
#define OPT3001_CONFIG_RN_2621LUX       0x06
#define OPT3001_CONFIG_RN_5242LUX       0x07
#define OPT3001_CONFIG_RN_10483LUX      0x08
#define OPT3001_CONFIG_RN_20966LUX      0x09
#define OPT3001_CONFIG_RN_41933LUX      0x0A
#define OPT3001_CONFIG_RN_83866LUX      0x0B
#define OPT3001_CONFIG_RN_AUTO          0x0C

#define OPT3001_CONFIG_RN_MASK          0xF000
#define OPT3001_CONFIG_RN_SHIFT         12

/* Conversion time field */

#define OPT3001_CONFIG_CT_100MS         0x0000
#define OPT3001_CONFIG_CT_800MS         0x0800

#define OPT3001_CONFIG_CT_MASK          0x0800

/* Mode of operation field */

#define OPT3001_CONFIG_M_SHDN           0x0000
#define OPT3001_CONFIG_M_SINGLE         0x0200
#define OPT3001_CONFIG_M_CONTINUOUS     0x0400

#define OPT3001_CONFIG_M_MASK           0x0600

/* Status flags */

#define OPT3001_STATUS_OVF              0x0100
#define OPT3001_STATUS_CRF              0x0080
#define OPT3001_STATUS_FH               0x0040
#define OPT3001_STATUS_FL               0x0020

#define OPT3001_STATUS_MASK             0x01E0

/* Interrupt latch field */

#define OPT3001_CONFIG_L_HYST           0x00
#define OPT3001_CONFIG_L_LATCH          0x10

#define OPT3001_CONFIG_L_MASK           0x10

/* Interrupt polarity field */

#define OPT3001_CONFIG_POL_LOW          0x00
#define OPT3001_CONFIG_POL_HIGH         0x08

#define OPT3001_CONFIG_POL_MASK         0x08

/* Mask exponent field */

#define OPT3001_CONFIG_ME               0x04

/* Fault count field */

#define OPT3001_CONFIG_FC_1             0x00
#define OPT3001_CONFIG_FC_2             0x01
#define OPT3001_CONFIG_FC_4             0x02
#define OPT3001_CONFIG_FC_8             0x03

#define OPT3001_CONFIG_FC_MASK          0x03

/* Sensor Configuration */
/* Provide an init stucture, returns OPT3001_OK or OPT3001_ERR */

uint8_t OPT3001_Init(OPT3001_InitTypeDef *OPT3001_InitStruct);

/* Data functions */

uint32_t OPT3001_GetData(void);

/* Status functions */

uint16_t OPT3001_GetStatus(void);


#endif