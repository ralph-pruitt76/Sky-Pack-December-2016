/* I2C bus low level driver
 * The I2C bus is used by the accelerometer/magnetometer, the gyro and the
 * fuel gauge */

#include "i2c_bus.h"


/* Write a 8-bit register to a chip */

void I2C_Write8bit(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t value)
{
  I2C_Write(DeviceAddr, RegAddr, &value, 1);
}

/* Read a 8-bit register from a chip */

uint8_t I2C_Read8bit(uint8_t DeviceAddr, uint8_t RegAddr)
{
  uint8_t value;
  I2C_Read(DeviceAddr, RegAddr, &value, 1);
  return value;
}

/* Union to get bytes from a 16-bit value, to help with big-endian
 * conversion */

union u16bit
{
  uint16_t w;
  uint8_t b[2];
};
 
/* Write a 16-bit big endian register to a chip after converting from
 * native little endian */

void I2C_Write16bitBE(uint8_t DeviceAddr, uint8_t RegAddr, uint16_t value)
{
  union u16bit val;
  uint8_t tmp;
  
  val.w = value;
  tmp = val.b[0];
  val.b[0] = val.b[1];
  val.b[1] = tmp;
  I2C_Write(DeviceAddr, RegAddr, val.b, 2);
}

/* Read a 16-bit big endian register from a chip and convert to native
 * little endian */

uint16_t I2C_Read16bitBE(uint8_t DeviceAddr, uint8_t RegAddr)
{
  union u16bit val;
  uint8_t tmp;
  
  I2C_Read(DeviceAddr, RegAddr, val.b, 2);
  tmp = val.b[0];
  val.b[0] = val.b[1];
  val.b[1] = tmp;
  return val.w;
}

/* Timeout variable for I2C access */

static volatile uint32_t I2C_Timeout = I2C_WAIT_LONG_TIMEOUT; 

/* Macro to simplify waiting for each of the I2C events */

#define WAIT_FOR_I2C_EVENT(ev)                          \
  do {                                                  \
    I2C_Timeout = I2C_WAIT_LONG_TIMEOUT;                \
    while (I2C_CheckEvent(I2C, (ev)) != SUCCESS) {      \
      if (--I2C_Timeout == 0) return I2C_FAIL;          \
    }                                                   \
  } while(0)

    
/**
  * @brief  Writes one or more bytes to a device on the I2C bus.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the device register to be written.
  * @param  pBuffer : pointer to the buffer containing the data to be written.
  * @param  NumBytesToWrite: number of bytes to write.
  * @retval Status
  */
    
uint32_t I2C_Write(uint8_t DeviceAddr, uint8_t RegAddr,
                   uint8_t* pBuffer, uint16_t NumBytesToWrite)
{
  /* Generate start condition */
  I2C_GenerateSTART(I2C, ENABLE);
  
  /* Wait until the start condition has been successfully released */
  WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_MODE_SELECT);
  
  /* Send and select slave address for writing */
  I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Transmitter);
  
  /* Wait until the slave acknowledges */
  WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

  /* Send the register address */
  I2C_SendData(I2C, RegAddr);
  
  /* Wait until transmission is started */
  WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTING);
  
  /* Loop until all data are transmitted */
  while (NumBytesToWrite)
  {
    /* Write the data byte */
    I2C_SendData(I2C, *pBuffer);

    /* Bump the pointer, decrement number of bytes left to read */
    pBuffer++;
    NumBytesToWrite--;
    
    /* Wait until transmission is done */
    WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
  }
  
  /* Send the stop condition after the byte is transmitted */
  I2C_GenerateSTOP(I2C, ENABLE);
  
  /* Everything went fine */
  return I2C_OK;
}

/**
  * @brief  Read one or more bytes from a device on the I2C bus.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the device register to read from.
  * @param  pBuffer : pointer to the buffer that receives the data.
  * @param  NumBytesToRead : number of bytes to read.
  * @retval Status
  */

uint32_t I2C_Read(uint8_t DeviceAddr, uint8_t RegAddr,
                  uint8_t* pBuffer, uint16_t NumBytesToRead)
{
  /* Generate start condition */
  I2C_GenerateSTART(I2C, ENABLE);
  
  /* Wait until the start condition has been successfully released */
  WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_MODE_SELECT);
  
  /* Send and select slave address for writing */
  I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Transmitter);
  
  /* Wait until the slave acknowledges */
  WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
  
  /* Send the register address */
  I2C_SendData(I2C, RegAddr);
  
  /* Wait until transmission is done */
  WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTED);

  /* Generate start condition */
  I2C_GenerateSTART(I2C, ENABLE);
  
  /* Wait until the start condition has been successfully released */
  WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_MODE_SELECT);
  
  /* Send and select slave address for reading */
  I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Receiver);
  
  /* Wait until the slave acknowledges */
  WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

  /* Loop until all data are received */
  while (NumBytesToRead)
  {
    /* Is this going to be the last byte? */
    if (NumBytesToRead == 1) {
      /* We won't send an acknowledgement */
      I2C_AcknowledgeConfig(I2C, DISABLE);
    } 
    
    /* Wait for a byte to arrive */
    WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_RECEIVED);
    
    /* Read a data byte */
    *pBuffer = I2C_ReceiveData(I2C);
    
    /* Bump the pointer, decrement number of bytes left to read */
    pBuffer++;
    NumBytesToRead--;
  }
  
  /* Send the stop condition after the byte is transmitted */
  I2C_GenerateSTOP(I2C, ENABLE);  

  /* Turn acknowledgement on again */
  I2C_AcknowledgeConfig(I2C, ENABLE);

  /* Everything went fine */
  return I2C_OK;
}  

/**
* @brief  Initializes the low level interface used to drive the I2C bus.
* @param  None
* @retval None
*/
void I2C_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
  
  /* Enable the I2C peripheral */
  RCC_APB1PeriphClockCmd(I2C_CLK, ENABLE);
  /* Enable SCK and SDA GPIO clocks */
  RCC_AHBPeriphClockCmd(I2C_SCK_GPIO_CLK | I2C_SDA_GPIO_CLK, ENABLE);
  
  /* Set SCK and SDA pin functions */
  GPIO_PinAFConfig(I2C_GPIO_PORT, I2C_SCK_SOURCE, I2C_SCK_AF);
  GPIO_PinAFConfig(I2C_GPIO_PORT, I2C_SDA_SOURCE, I2C_SDA_AF);
  
  /* I2C SCK and SDA pin configuration */
  GPIO_InitStructure.GPIO_Pin = I2C_SCK_PIN | I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);
  
  /* I2C configuration */
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  
  /* Apply LSM303DLHC_I2C configuration after enabling it */
  I2C_Init(I2C, &I2C_InitStructure);
  
  /* LSM303DLHC_I2C Peripheral Enable */
  I2C_Cmd(I2C, ENABLE);
}  
