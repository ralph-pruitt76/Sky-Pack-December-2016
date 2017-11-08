/* I2C bus low level header file
 * The I2C bus is used by the accelerometer/magnetometer, the gyro and the
 * fuel gauge */

#ifndef I2C_BUS_H_
#define I2C_BUS_H_

#include "stm32l1xx.h"
#include "usart.h"

/* OK and Fail definitions */

#define I2C_OK                  ((uint32_t) 1)
#define I2C_FAIL                ((uint32_t) 0)

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */

#define I2C_WAIT_FLAG_TIMEOUT   ((uint32_t)0x1000)
#define I2C_WAIT_LONG_TIMEOUT   ((uint32_t)(10 * I2C_WAIT_FLAG_TIMEOUT))  

/* I2C Interface pins */

#define I2C                     I2C1
#define I2C_CLK                 RCC_APB1Periph_I2C1
#define I2C_GPIO_PORT           GPIOB
#define I2C_SPEED               20000

#define I2C_SCK_PIN             GPIO_Pin_6                  /* PB6 */
#define I2C_SCK_GPIO_CLK        RCC_AHBPeriph_GPIOB
#define I2C_SCK_SOURCE          GPIO_PinSource6
#define I2C_SCK_AF              GPIO_AF_I2C1

#define I2C_SDA_PIN             GPIO_Pin_7                  /* PB7 */
#define I2C_SDA_GPIO_CLK        RCC_AHBPeriph_GPIOB
#define I2C_SDA_SOURCE          GPIO_PinSource7
#define I2C_SDA_AF              GPIO_AF_I2C1


/* Low level initialization of the I2C bus */

HAL_StatusTypeDef I2C_LowLevel_Init(void);

/* Generic read and write funtions */

HAL_StatusTypeDef I2C_Write(uint8_t DeviceAddr, uint8_t RegAddr,
                   uint8_t* pBuffer, uint16_t NumBytesToWrite);
HAL_StatusTypeDef I2C_Read(uint8_t DeviceAddr, uint8_t RegAddr,
                  uint8_t* pBuffer, uint16_t NumBytesToRead);

/* Write a 8-bit register to a chip */

HAL_StatusTypeDef I2C_Write8bit(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t value);

/* Read a 8-bit register from a chip */

uint8_t I2C_Read8bit(uint8_t DeviceAddr, uint8_t RegAddr);

/* Write a 16-bit big endian register to a chip after converting from
 * native little endian */

HAL_StatusTypeDef I2C_Write16bitBE(uint8_t DeviceAddr, uint8_t RegAddr, uint16_t value);

/* Read a 16-bit big endian register from a chip and convert to native
 * little endian */

uint16_t I2C_Read16bitBE(uint8_t DeviceAddr, uint8_t RegAddr);

HAL_StatusTypeDef SkyPack_TestI2C( void );
HAL_StatusTypeDef WAIT_FOR_I2C_EVENT(uint32_t ev);
HAL_StatusTypeDef I2C_LowLevel_DeInit(void);

#endif
