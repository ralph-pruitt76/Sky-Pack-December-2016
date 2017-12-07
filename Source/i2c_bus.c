/* I2C bus low level driver
 * The I2C bus is used by the accelerometer/magnetometer, the gyro and the
 * fuel gauge */

#include "i2c_bus.h"
#include "sys_ctrl.h"
#include "app_data.h"
#include "miscRoutines.h"
#include "ErrCodes.h"
#include "stdbool.h"
#include <string.h>
#include <stdio.h>

/**
  * @brief  Attempts to Clear I2C channel Data Pin by pulsing SDA Line 9 times.
  * @param None.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to I2C success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  */
HAL_StatusTypeDef SkyPack_I2CRepair( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  HAL_StatusTypeDef Status;
  int x, loop_cnt;
  char tempBffr2[8];
  
  Status = HAL_OK;

  GPIO_InitStructure.GPIO_Pin = I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Wait for Hardware to stabilize....10ms
  delay_10ms();
  
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Wait for Hardware to stabilize....100ms
  delay_100ms();
  
  for (loop_cnt=0; loop_cnt<I2C_LOOPCNT; loop_cnt++)
  {
    // Print Loop Count
    sprintf( (char *)tempBffr2, "**%02d.", loop_cnt);
    SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
    
    // Now...Pulse SDA Pin I2C_CLKRPRCNT Times.
    for (x=0; x<I2C_CLKRPRCNT; x++)
    {
      SkyPack_gpio_Off(gI2C_CLK);  // Set Clock Low.
      delay_10ms();               // Wait 10ms;
      SkyPack_gpio_On(gI2C_CLK);   // Set Clock High.
      delay_10ms();               // Wait 10ms;
    } // EndFor (x=0; x<I2C_CLKRPRCNT; x++)
    
    // Finally Reset Clock LOW.
    SkyPack_gpio_Off(gI2C_CLK);  // Set Clock Low.
    
    // Wait for Hardware to stabilize....100ms
    delay_100ms();;
    
    // Test Data Pin State.
    if ( HAL_GPIO_ReadPin( GPIOB, I2C_SDA_Pin) == GPIO_PIN_RESET)
    {
      Status = HAL_ERROR;
    }
    else
    {
      // If High, Then we have been succesful. Time to Indicate Repaired and Init I2C BUS.
      //SkPck_ErrCdLogErrCd( REPAIR_I2C, MODULE_i2c );
      // Enable all I2C Sensors.
      //Set_DriverStates( I2C_STATE, DRIVER_ON );
      //Set_DriverStates( IMU_STATE_TASK, DRIVER_ON );
      //Set_DriverStates( IRRADIANCE_MNTR_TASK, DRIVER_ON );
      //Set_DriverStates( PRESSURE_MNTR_TASK, DRIVER_ON );
      // Now Reinit I2C Bus.
      //I2C_LowLevel_Init();
      return HAL_OK;
    }
    delay_100ms();
  } // EndFor (loop_cnt=0; loop_cnt<I2C_LOOPCNT; loop_cnt++)
  return Status;
}

/**
  * @brief  Tests I2C channel and sets error codes if failed.
  * @param None.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to I2C success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     I2C is busy.
  *                                HAL_TIMEOUT:  I2C timed out.
  */
HAL_StatusTypeDef SkyPack_TestI2C( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;

  GPIO_InitStructure.GPIO_Pin = I2C_SCL_Pin|I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Wait for Hardware to stabilize....100ms
  delay_100ms();;
  
  // Test I2C Clock
  if ( HAL_GPIO_ReadPin( GPIOB, I2C_SCL_Pin) == GPIO_PIN_RESET )
  {
    // If low....Then SCLK has failed...Record Error.
    SkPck_ErrCdLogErrCd( ERROR_I2C_SCLK, MODULE_i2c );
    Set_DriverStates( I2C_STATE, DRIVER_OFF );
    Status = HAL_ERROR;
    
    // Time to test SDAT
    if ( HAL_GPIO_ReadPin( GPIOB, I2C_SDA_Pin) == GPIO_PIN_RESET)
    {
      // If low....Then SDAT has failed...Record Error.
      SkPck_ErrCdLogErrCd( ERROR_I2C_SDAT, MODULE_i2c );
    }
  }
  else
  {
    // Passed, Time to test SDAT
    if ( HAL_GPIO_ReadPin( GPIOB, I2C_SDA_Pin) == GPIO_PIN_RESET)
    {
      // If low....Then SDAT has failed...Record Error.
      SkPck_ErrCdLogErrCd( ERROR_I2C_SDAT, MODULE_i2c );
      Set_DriverStates( I2C_STATE, DRIVER_OFF );
      Status = HAL_ERROR;
    }
    else
    {
      Set_DriverStates( I2C_STATE, DRIVER_ON );
      Status = HAL_OK;
    }
  } 
  
  return Status;
}

/* Write a 8-bit register to a chip */

HAL_StatusTypeDef I2C_Write8bit(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t value)
{
  return (I2C_Write(DeviceAddr, RegAddr, &value, 1));
}

/* Read a 8-bit register from a chip */

uint8_t I2C_Read8bit(uint8_t DeviceAddr, uint8_t RegAddr)
{
  uint8_t value;
  if (I2C_Read(DeviceAddr, RegAddr, &value, 1) == HAL_OK)
    return value;
  else
    return 0xff;
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

HAL_StatusTypeDef I2C_Write16bitBE(uint8_t DeviceAddr, uint8_t RegAddr, uint16_t value)
{
  union u16bit val;
  uint8_t tmp;
  
  val.w = value;
  tmp = val.b[0];
  val.b[0] = val.b[1];
  val.b[1] = tmp;
  return (I2C_Write(DeviceAddr, RegAddr, val.b, 2));
}

/* Read a 16-bit big endian register from a chip and convert to native
 * little endian */

uint16_t I2C_Read16bitBE(uint8_t DeviceAddr, uint8_t RegAddr)
{
  union u16bit val;
  uint8_t tmp;
  
  if (I2C_Read(DeviceAddr, RegAddr, val.b, 2) == HAL_OK)
  {
    tmp = val.b[0];
    val.b[0] = val.b[1];
    val.b[1] = tmp;
    return val.w;
  }
  else
    return 0xffff;
}

/* Timeout variable for I2C access */

static volatile uint32_t I2C_Timeout = I2C_WAIT_LONG_TIMEOUT; 

HAL_StatusTypeDef WAIT_FOR_I2C_EVENT(uint32_t ev)
{
  uint32_t      I2C_Timeout;
  
  I2C_Timeout = I2C_WAIT_LONG_TIMEOUT;
  
  while (I2C_CheckEvent(I2C, (ev)) != SUCCESS) {
    if (--I2C_Timeout == 0) return HAL_TIMEOUT; 
  }  
  return HAL_OK;
}

/* Macro to simplify waiting for each of the I2C events */
/*#define WAIT_FOR_I2C_EVENT(ev)                          \
  do {                                                  \
    I2C_Timeout = I2C_WAIT_LONG_TIMEOUT;                \
    while (I2C_CheckEvent(I2C, (ev)) != SUCCESS) {      \
      if (--I2C_Timeout == 0) return I2C_FAIL;          \
    }                                                   \
  } while(0)*/

    
/**
  * @brief  Writes one or more bytes to a device on the I2C bus.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the device register to be written.
  * @param  pBuffer : pointer to the buffer containing the data to be written.
  * @param  NumBytesToWrite: number of bytes to write.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to I2C success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     I2C is busy.
  *                                HAL_TIMEOUT:  I2C timed out.
  */
    
HAL_StatusTypeDef I2C_Write(uint8_t DeviceAddr, uint8_t RegAddr,
                   uint8_t* pBuffer, uint16_t NumBytesToWrite)
{
  static bool busy_flg = false;
  
  if (busy_flg == false)
  {
    // Set busy flag to keep a second operation from starting
    busy_flg = true;
    // Turn on Status LED.
    SkyPack_gpio_On(STATUS_LED);
    
    /* Generate start condition */
    I2C_GenerateSTART(I2C, ENABLE);
    
    /* Wait until the start condition has been successfully released */
    //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_MODE_SELECT);
    if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_MODE_SELECT) == HAL_TIMEOUT)
    {
      // Clear busy flag
      busy_flg = false;
      // Declare Error.
      return HAL_TIMEOUT;
    }
    
    /* Send and select slave address for writing */
    I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Transmitter);
    
    /* Wait until the slave acknowledges */
    //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == HAL_TIMEOUT)
    {
      // Clear busy flag
      busy_flg = false;
      // Declare Error.
      return HAL_TIMEOUT;
    }

    /* Send the register address */
    I2C_SendData(I2C, RegAddr);
    
    /* Wait until transmission is started */
    //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTING);
    if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTING) == HAL_TIMEOUT)
    {
      // Clear busy flag
      busy_flg = false;
      // Declare Error.
      return HAL_TIMEOUT;
    }
    
    /* Loop until all data are transmitted */
    while (NumBytesToWrite)
    {
      /* Write the data byte */
      I2C_SendData(I2C, *pBuffer);

      /* Bump the pointer, decrement number of bytes left to read */
      pBuffer++;
      NumBytesToWrite--;
      
      /* Wait until transmission is done */
      //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
      if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == HAL_TIMEOUT)
      {
        // Clear busy flag
        busy_flg = false;
        // Declare Error.
        return HAL_TIMEOUT;
      }
    }
    
    /* Send the stop condition after the byte is transmitted */
    I2C_GenerateSTOP(I2C, ENABLE);
    
    // Turn Off Status LED.
    SkyPack_gpio_Off(STATUS_LED);
    
    // Clear busy flag
    busy_flg = false;

    /* Everything went fine */
    return HAL_OK;
  }
  else
    return HAL_BUSY;
}

/**
  * @brief  Read one or more bytes from a device on the I2C bus.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the device register to read from.
  * @param  pBuffer : pointer to the buffer that receives the data.
  * @param  NumBytesToRead : number of bytes to read.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to I2C success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     I2C is busy.
  *                                HAL_TIMEOUT:  I2C timed out.
 */

HAL_StatusTypeDef I2C_Read(uint8_t DeviceAddr, uint8_t RegAddr,
                  uint8_t* pBuffer, uint16_t NumBytesToRead)
{
  static bool busy_flg = false;
  
  if (busy_flg == false)
  {
    // Set busy flag to keep a second operation from starting
    busy_flg = true;
    // Turn on Status LED.
    SkyPack_gpio_On(STATUS_LED);
    
    /* Generate start condition */
    I2C_GenerateSTART(I2C, ENABLE);
    
    /* Wait until the start condition has been successfully released */
    if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_MODE_SELECT) == HAL_TIMEOUT)
    {
      // Clear busy flag
      busy_flg = false;
      // Declare Error.
      return HAL_TIMEOUT;
    }
    
    /* Send and select slave address for writing */
    I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Transmitter);
    
    /* Wait until the slave acknowledges */
    //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == HAL_TIMEOUT)
    {
      // Clear busy flag
      busy_flg = false;
      // Declare Error.
      return HAL_TIMEOUT;
    }
    
    /* Send the register address */
    I2C_SendData(I2C, RegAddr);
    
    /* Wait until transmission is done */
    //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == HAL_TIMEOUT)
    {
      // Clear busy flag
      busy_flg = false;
      // Declare Error.
      return HAL_TIMEOUT;
    }

    /* Generate start condition */
    I2C_GenerateSTART(I2C, ENABLE);
    
    /* Wait until the start condition has been successfully released */
    //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_MODE_SELECT);
    if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_MODE_SELECT) == HAL_TIMEOUT)
    {
      // Clear busy flag
      busy_flg = false;
      // Declare Error.
      return HAL_TIMEOUT;
    }
    
    /* Send and select slave address for reading */
    I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Receiver);
    
    /* Wait until the slave acknowledges */
    //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == HAL_TIMEOUT)
    {
      // Clear busy flag
      busy_flg = false;
      // Declare Error.
      return HAL_TIMEOUT;
    }

    /* Loop until all data are received */
    while (NumBytesToRead)
    {
      /* Is this going to be the last byte? */
      if (NumBytesToRead == 1) {
        /* We won't send an acknowledgement */
        I2C_AcknowledgeConfig(I2C, DISABLE);
      } 
      
      /* Wait for a byte to arrive */
      //WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_RECEIVED);
      if (WAIT_FOR_I2C_EVENT(I2C_EVENT_MASTER_BYTE_RECEIVED) == HAL_TIMEOUT)
      {
        // Clear busy flag
        busy_flg = false;
        // Declare Error.
        return HAL_TIMEOUT;
      }
      
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

    // Turn Off Status LED.
    SkyPack_gpio_Off(STATUS_LED);
    
    // Clear busy flag
    busy_flg = false;

    /* Everything went fine */
    return HAL_OK;
  }
  else
    return HAL_BUSY;
}  

/**
* @brief  Initializes the low level interface used to drive the I2C bus.
* @param  None
* @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to I2C success.
*/
HAL_StatusTypeDef I2C_LowLevel_Init(void)
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
  
  return HAL_OK;
}  

/**
* @brief  De-Initializes the low level interface used to drive the I2C bus.
* @param  None
* @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to I2C success.
*/
HAL_StatusTypeDef I2C_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Apply LSM303DLHC_I2C configuration after enabling it */
  I2C_DeInit(I2C);

  /* I2C SCK pin configuration */
  // I2C_SCK_PIN  pin configuration.
  GPIO_InitStructure.GPIO_Pin = I2C_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);

  /* I2C SDA pin configuration */
  // I2C_SDA_PIN  pin configuration.
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);
  
  /* LSM303DLHC_I2C Peripheral Enable */
  //I2C_Cmd(I2C, ENABLE);
  
  return HAL_OK;
}  
