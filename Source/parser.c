/**
  ******************************************************************************
  * File Name          : parser.c
  * Description        : This file provides code that parses the passed string and 
  *                      performs the requested operation. It then returns a string
  *                      hardware based on the LPS22HB chip.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 WeatherCloud
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
#include "parser.h"
#include "ErrCodes.h"
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include "wwdg.h"
#include "misc_sense.h"
//#include "Calibration.h"

// Enums
typedef enum 
{
  NOT_INIT = 0,
  AVAILABLE = 1,
  BUSY = 2
} ParseTskFlg;

HAL_StatusTypeDef ReadIMUSensorsLocal(void);

static bool Bypass = false;

// Parser Structure for tasks.
struct
{
  char          tempBuffer[BUFFER_SIZE];
  ParseTskFlg   ParseFlg;
} static ParseString;

/* Parser functions */

/**
  * @brief  This routine initializes the Parse Task Structure.
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  */
char *CleanString( char *mntrCmd )
{
  static char tempString[30];
  char *tempPtr;
  int StringCnt = 0;
  tempPtr = &tempString[0];
  
  while( *mntrCmd != 0x00 )
  {
    if (*mntrCmd == 0x08)
    {
      if (StringCnt > 0)
      {
        tempPtr--;
        StringCnt--;
      }
    }
    else
    {
      *tempPtr++ = *mntrCmd;
      StringCnt++;
    }
    *mntrCmd++;
  }
  *tempPtr++ = 0x00;
  return tempString;
}


/**
  * @brief  This routine initializes the Parse Task Structure.
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  */
HAL_StatusTypeDef SkyBrd_ParserInit( void )
{
  ParseString.ParseFlg = AVAILABLE;
  return HAL_OK;
}

/**
  * @brief  This routine handles the operation of setting up a Parse Event.
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef SkyBrd_ParserTsk(char *tempBffr)
{
  // Test ParseFlg.
  if (ParseString.ParseFlg == BUSY)
    return HAL_BUSY;
  else if (ParseString.ParseFlg == NOT_INIT)
    return HAL_ERROR;
  // Next Lets make sure passed string is not too big.
//  if (strlen((char *)tempBffr) >= BUFFER_SIZE)
  if (strlen(tempBffr) >= BUFFER_SIZE)
    return HAL_ERROR;
  // Copy String into Structure and set as busy.
//  strcpy( (char *)ParseString.tempBuffer, (char *)tempBffr);
  strcpy( ParseString.tempBuffer, tempBffr);
  ParseString.ParseFlg = BUSY;
  return HAL_OK;
}

/**
  * @brief  This routine handles the operation of processing a Parse Event.
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef SkyBrd_ProcessParserTsk( void )
{
  HAL_StatusTypeDef Status;

  // Test ParseFlg and process.
  if (ParseString.ParseFlg == BUSY)
  {
    Status = SkyBrd_ParseString(ParseString.tempBuffer, true);
    ParseString.ParseFlg = AVAILABLE;
    return Status;
  }
  else
    return HAL_OK;
}

union u3DVector
{
  struct
  {
    int16_t             x;
    int16_t             y;
    int16_t             z;
  } named;
  int16_t               indexed[3];
};

struct {
  union u3DVector     accel;
  union u3DVector     gyro;
  union u3DVector     mag;
} imu;
int16_t         temperature;
int16_t         pressure;
int32_t         irradiance;
float           value;
struct {
  uint16_t            event_freq;
  uint16_t            swept_idx;
  uint16_t            swept_level;
} cap;

/* Parser function */

/**
  * @brief  This routine parses the passed string and performs the passed operation
  * @param  *tempBffr: String to be parsed.
  * @param  bool BLE_Flag: Flag to indicate a BLE Task.
  *                      true(1): BLE Task of Monitor Event.
  *                      true(0): Normal Monitor Event.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef SkyBrd_ParseString(char *tempBffr, bool BLE_Flag)
{
#define RECEIVE_SZ      30
  uint16_t DriverStatus;
  int8_t tempBffr2[120];
  int8_t tempBffr3[10];
//    int8_t* BufferPntr;
    HAL_StatusTypeDef Status;
//    HAL_StatusTypeDef Save_Status;
    uint8_t Size;
    int Address;
    int num_bytes;
    int num_bytes_received;
    uint8_t i2cData[80];
    uint8_t temp_reg;
    int x;
    int Error;
//    int y;
   //Voltage VMeasure, VMeasureScaled;
    //Current CMeasure, CMeasureScaled;
    //Power PMeasure, PMeasureScaled;
    //Temperature TMeasure, TMeasureScaled;
    //Humidity HMeasure, HMeasureScaled;
    //RGBInitialize RGBMeasure;
    //RGBIdent IDMeasure;
    //RGBStatus RGBSMeasure;
    //RGBLight RGBValues;
//    char uuid[10];
//    float Scale, Offset;
//    PRStatus PRMeasure;
    //PRPressure PRPMeasure, PRPMeasureScaled;
    //BinString RSFFTBins;
    //GridEye     GridMeasure, GridMeasureScaled;
//    uint32_t Err_code;
//    uint8_t op_mode, ds_range, adc_rsl, sync, cmp_adjst, cmp_offst, int_assgn, int_persist, cnvrsn_int;
    int new_value, flag;
    char* tempPstr;
    char tempstr[20];

    Size = strlen((char *)tempBffr);
    
    // Test Bypass. If set, then we are in streaming mode.
    if ( Bypass )
    {
      if (tempBffr[0] == 0x1B)
      {
        Bypass = false;
        strcpy( (char *)tempBffr2, "\r\n\r\n T........TERMINATING MONITOR MODE.........\r\n\r\n> ");
        Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
        if (Status != HAL_OK)
          return Status;
      }
      else
      {
        // Transmit Character to BGM111.
        BGM111_Transmit(1, (uint8_t *)tempBffr);
      }
    }// EndIf ( Bypass )
    else
    {
      // Normal Mode
    
            // We have a good Tasking String. Time to determine action.
            switch( tempBffr[0] )
            {

//**************************************************************************************************
            case '0':
              // Dump accelerometer data. 
              // Read IMU Sensors
              Status = ReadIMUSensorsLocal();
              if (Status == HAL_OK)
              {
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Dump Accelerometer Data...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     X DATA: %05d (0.001g)\r\n", imu.accel.named.x );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Y DATA: %05d (0.001g)\r\n", imu.accel.named.y );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Z DATA: %05d (0.001g)\r\n", imu.accel.named.z );
              }
              else
                break;
              break;
//**************************************************************************************************
            case '1':
              // Dump Gyro data. 
              // Read IMU Sensors
              Status = ReadIMUSensorsLocal();
              if (Status == HAL_OK)
              {
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Dump Gyro Data...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     X DATA: %05d (0.1dps)\r\n", imu.gyro.named.x );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Y DATA: %05d (0.1dps)\r\n", imu.gyro.named.y );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Z DATA: %05d (0.1dps)\r\n", imu.gyro.named.z );
              }
              else
                break;
              break;
//**************************************************************************************************
            case '2':
              // Dump Magnetometer data. 
              // Read IMU Sensors
              Status = ReadIMUSensorsLocal();
              if (Status == HAL_OK)
              {
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Dump Magnetometer Data...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     X DATA: %05d (mgauss)\r\n", imu.mag.named.x );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Y DATA: %05d (mgauss)\r\n", imu.mag.named.y );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Z DATA: %05d (mgauss)\r\n", imu.mag.named.z );
              }
              else
                break;
              break;
//**************************************************************************************************
            case 'A':
              // Read Pressure Sensor. 
               Status = LPS25HB_GetPressure(&value);
              if (Status == HAL_OK)
              {
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Pressure Sensor...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                // Now Display.
                sprintf( (char *)tempBffr2, "     Pressure: %5.1fmbar\r\n", value);
              }
              else
                break;
              break;
//**************************************************************************************************
            case 'G':
              // Read Temperature Sensor. 
              Status = LPS25HB_GetTemperature(&value);
              if (Status == HAL_OK)
              {
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Temperature Sensor...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                // Now calculate Celcius and Farenheit Temp.
                sprintf( (char *)tempBffr2, "     Temperature: %3.1fC\r\n", value);
              }
              else
                break;
              break;
//**************************************************************************************************
            case 'H':
              // Read Irradiance Light Sensor. 
              irradiance = OPT3001_GetData();
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
              // Send string to UART..
              strcpy( (char *)tempBffr2, "Irradiance Light Sensor...\r\n");
              Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
              if (Status != HAL_OK)
                return Status;
              // Now calculate Celcius and Farenheit Temp.
              sprintf( (char *)tempBffr2, "     Irradiance: %d(100lx)\r\n", irradiance);
              break;    
//**************************************************************************************************
            case 'C':
              // Read Cap Sense Event Frequency. 
              cap.event_freq = GetCapSenseEventFreq();
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              // Send string to UART..
              strcpy( (char *)tempBffr2, "Cap Sense Event Frequency...\r\n");
              Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
              if (Status != HAL_OK)
                return Status;
              // Now calculate Celcius and Farenheit Temp.
              sprintf( (char *)tempBffr2, "     Frequency: %d(Hz/100)\r\n", cap.event_freq);
              break;
//**************************************************************************************************
            case 'D':
              // Read Swept Frequency. 
              cap.swept_idx = GetSweptFreqHighIdx();
              cap.swept_level = GetSweptFreqHighLevel();
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              // Send string to UART..
              strcpy( (char *)tempBffr2, "Swept Frequency...\r\n");
              Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
              if (Status != HAL_OK)
                return Status;

              strcpy( (char *)tempBffr2, "     Swept Frequency Sensor frequency index with the \r\n");
              Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
              if (Status != HAL_OK)
                return Status;
              sprintf( (char *)tempBffr2, "          highest output level: %d\r\n", cap.swept_idx);
              Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
              if (Status != HAL_OK)
                return Status;

              sprintf( (char *)tempBffr2, "     Swept Frequency Sensor highest level: %dHz\r\n", cap.swept_level);
 
              break;
//**************************************************************************************************
            case 'T':
              // TEST CMDS. 
              // Test Size to make sure we have enough Characters for this operation
              if (Size <= 1)
                strcpy( (char *)tempBffr2, "T ERROR: Not a legal command.\r\n");
              else
              {
                switch( tempBffr[1] )
                {
//++++++++++++++++++++++++++++++++++++++++++  I2C Commands.
                  case 'I':
                    strcpy( (char *)tempBffr2, "TI CMD: I2C CMDS NOT IMPLEMENTED\r\n");
                    // I2C Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    if (Size < 9)
                    {
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      
                      strcpy( (char *)tempBffr2, "TI SYNTAX ERROR: Not correct format.\r\n");
                    }
                    else
                    {
                      switch( tempBffr[2] )
                      {
//------------------
                        case 'S':
                          //I2C Send Command.
                          // Step 1. Validate format.
                          if( (tempBffr[3]!=':') ||
                              (tempBffr[6]!='.') )
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                           
                            strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            // 2. Grab Address and validate a legal number
                            tempBffr3[0] = tempBffr[4];
                            tempBffr3[1] = tempBffr[5];
                            tempBffr3[2] = 0x00;
                            if (isHexNum( (char *)tempBffr3 ) == 0)
                              strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Address not HEX Value.\r\n");
                            else
                            {
                              // Legal Address. Save it as value
                              Address = hatoi( (char *)tempBffr3 );
                              // 3. Now get the number of bytes of data from field.
//                              sprintf( (char *)tempBffr2, "TIS: Good Address: %x.\r\n", Address);
                              tempBffr3[0] = tempBffr[7];
                              tempBffr3[1] = tempBffr[8];
                              tempBffr3[2] = 0x00;
                              if (isHexNum( (char *)tempBffr3 ) == 0)
                                strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Number of Bytes not HEX Value.\r\n");
                              else
                              {
                                // Legal NUMBER BYTES. Save it as value
                                num_bytes = hatoi( (char *)tempBffr3 );
                                // 4. Test num_bytes. If Zero, We are done
                                if (num_bytes == 0)
                                {
                                  sprintf( (char *)tempBffr2, "TIS: GOOD CMD: %x.\r\n", Address);
                                }
                                else
                                {
                                  // 5. Time to get all the data.
                                  Error = 0;
                                  for (x=0; x<num_bytes; x++)
                                  {
                                    tempBffr3[0] = tempBffr[10+x*3];
                                    tempBffr3[1] = tempBffr[11+x*3];
                                    tempBffr3[2] = 0x00;
                                    if (isHexNum( (char *)tempBffr3 ) == 0)
                                    {
                                      strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Address not HEX Value.\r\n");
                                      Error = 1;
                                      break;
                                    }
                                    else
                                    {
                                      i2cData[x] =  hatoi( (char *)tempBffr3 );
                                    } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)
                                  } //EndFor (x=0; x<num_bytes; x++)
                                } //EndElse (num_bytes == 0)
                                if (Error==0)
                                {
                                  sprintf( (char *)tempBffr2, "TIS: GOOD CMD: %x.", Address);
                                }
                                for(x=0; x<num_bytes; x++)
                                {
                                  sprintf( (char *)tempBffr3, "%x.", i2cData[x]);
                                  strcat( (char *)tempBffr2, (char *)tempBffr3 );
                                }
                                strcat( (char *)tempBffr2, "\r\n" );
                                // 6. Time to send Command and collect status.
//                                Status =  RoadBrd_I2C_Master_Transmit((uint16_t)Address, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
                                Status =  I2C_Write((uint8_t)Address, (uint8_t)i2cData[0], (uint8_t *)&(i2cData[1]), (uint16_t)(num_bytes-1));
                              } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...NUMBER BYTES
                              
                            } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...Address
                            
                          } //EndElse ( (tempBffr[2]!=':') || (tempBffr[5]!='.') )
                      
                          break;
//------------------
                        case 'R':
                          //I2C Receive Command
                          // Step 1. Validate format.
                          if( (tempBffr[3]!=':') ||
                              (tempBffr[6]!='.') ||
                              (tempBffr[9]!='.')  )
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            // 2. Grab Address and validate a legal number
                            tempBffr3[0] = tempBffr[4];
                            tempBffr3[1] = tempBffr[5];
                            tempBffr3[2] = 0x00;
                            if (isHexNum( (char *)tempBffr3 ) == 0)
                              strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Address not HEX Value.\r\n");
                            else
                            {
                              // Legal Address. Save it as value
                              Address = hatoi( (char *)tempBffr3 );
                              // 3. Now get the number of bytes of data from field.
                              tempBffr3[0] = tempBffr[7];
                              tempBffr3[1] = tempBffr[8];
                              tempBffr3[2] = 0x00;
                              if (isHexNum( (char *)tempBffr3 ) == 0)
                                strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Number of Bytes not HEX Value.\r\n");
                              else
                              {
                                // Legal NUMBER BYTES. Save it as value
                                num_bytes = hatoi( (char *)tempBffr3 );
                                
                                // 3a. Now get the number of bytes Received of data from field.
                                tempBffr3[0] = tempBffr[10];
                                tempBffr3[1] = tempBffr[11];
                                tempBffr3[2] = 0x00;
                                if (isHexNum( (char *)tempBffr3 ) == 0)
                                  strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Number of Bytes RECEIVED not HEX Value.\r\n");
                                else
                                {
                                  // Legal NUMBER BYTES. Save it as value
                                  num_bytes_received = hatoi( (char *)tempBffr3 );
                                  // 4. Test num_bytes. If Zero, We are done
                                  if (num_bytes == 0)
                                  {
                                    sprintf( (char *)tempBffr2, "TIR: GOOD CMD: %x.\r\n", Address);
                                  }
                                  else
                                  {
                                    // 5. Time to get all the data.
                                    Error = 0;
                                    for (x=0; x<num_bytes; x++)
                                    {
                                      tempBffr3[0] = tempBffr[13+x*3];
                                      tempBffr3[1] = tempBffr[14+x*3];
                                      tempBffr3[2] = 0x00;
                                      if (isHexNum( (char *)tempBffr3 ) == 0)
                                      {
                                        strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Data not HEX Value.\r\n");
                                        Error = 1;
                                        break;
                                      }
                                      else
                                      {
                                        i2cData[x] =  hatoi( (char *)tempBffr3 );
                                      } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)
                                    } //EndFor (x=0; x<num_bytes; x++)
                                  } //EndElse (num_bytes == 0)
                                  if (Error==0)
                                  {
                                    sprintf( (char *)tempBffr2, "TIR: GOOD CMD: %x.", Address);
                                  }
                                  for(x=0; x<num_bytes; x++)
                                  {
                                    sprintf( (char *)tempBffr3, "%x.", i2cData[x]);
                                    strcat( (char *)tempBffr2, (char *)tempBffr3 );
                                  }
                                  strcat( (char *)tempBffr2, "\r\n" );
                                  // 6. Time to get response.
                                  temp_reg = i2cData[0];
//                                  Status =  RoadBrd_I2C_Master_Receive((uint16_t)Address, i2cData, (uint16_t)num_bytes_received, I2C_TIMEOUT);
                                  Status =  I2C_Read((uint8_t)Address, temp_reg, (uint8_t*)i2cData, (uint16_t)num_bytes_received);
                                  // 8. IfGood report, Need to Output Data.
                                  if (Status == HAL_OK)
                                  {
                                    // Send string to UART..
                                    Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                                    if (Status != HAL_OK)
                                      return Status;
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "     DATA: " );
                                    for(x=0; x<num_bytes_received; x++)
                                    {
                                      sprintf( (char *)tempBffr3, "%x.", i2cData[x]);
                                      strcat( (char *)tempBffr2, (char *)tempBffr3 );
                                    }
                                    strcat( (char *)tempBffr2, "\r\n" );
                                  }
                                }
                              } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...NUMBER BYTES
                              
                            } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...Address
                            
                          } //EndElse ( (tempBffr[2]!=':') || (tempBffr[5]!='.') )
                      
                          break;
                        default:
                          strcpy( (char *)tempBffr2, "TI CMD: ERROR! Not a legal command.\r\n");
                          break;
                      } //EndSwitch ( tempBffr[2] )
                    } //EndElse (Size < 9)
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Calibration Commands.
                  case 'C':
                    Status = HAL_OK;
                    strcpy( (char *)tempBffr2, "TC CMD: Calibration CMDS NOT IMPLEMENTED\r\n");
/*                    if (Size == 2)
                    {
                      //------------------ TC Command: Dump Calibration Settings.      
                      // Read Cool Eye/Grid Eye Values.....
                      if ( Get_DriverStates( GRIDEYE_MNTR_TASK ))
                      {
                        Status = RoadBrd_GridEye_ReadValues( &GridMeasure );
                      }
                      else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
                      {
                        Status = RoadBrd_CoolEye_ReadValues( &GridMeasure );
                      }
                      else
                        Status = HAL_ERROR;
                      
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      
                      if (Status == HAL_OK)
                      {
                        // OK Next Sensor.
                        // Read Temperature sensor and return results....Temperature Sensor U10(PCT2075GVJ).  Addr: 0x94
                        Status = RoadBrd_ReadTemp( &TMeasure );
                        if (Status == HAL_OK)
                        {
                          // OK Next Sensor.
                          // Read Humidity Sensor sensor and return Humidity results....
                          Status = RoadBrd_Humidity_ReadHumidity( &HMeasure );
                          if (Status == HAL_OK)
                          {
                            // OK Next Sensor.
                            //Status = RoadBrd_Barometer_Status( &PRMeasure );
                            Status = RoadBrd_Baro_ReadPressure( &PRPMeasure );
                            if (Status == HAL_OK)
                            {
                              //sprintf( (char *)tempBffr2, "Driver Status: %04x\r\n", DriverStatus );
                              sprintf( (char *)tempBffr2, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n", (char *)GridMeasure.GridEye1.TempC,
                                      (char *)GridMeasure.GridEye2.TempC,
                                      (char *)GridMeasure.GridEye3.TempC,
                                      (char *)GridMeasure.GridEye4.TempC,
                                      (char *)GridMeasure.GridEye5.TempC,
                                      (char *)GridMeasure.GridEye6.TempC,
                                      (char *)GridMeasure.GridEye7.TempC,
                                      (char *)GridMeasure.GridEye8.TempC,
                                      (char *)GridMeasure.Thermistor.TempC,
                                      (char *)TMeasure.TempC,
                                      (char *)HMeasure.Humidity,
                                      (char *)PRPMeasure.Pressure);
                              // Send string to UART..
#ifdef NUCLEO
                              Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                              Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                              if (Status != HAL_OK)
                                return Status;
                              // NOW, Build Data String..
                              sprintf( (char *)tempBffr2, "COMPLETE" );
                            } // Endif (Status == HAL_OK) RoadBrd_Baro_ReadPressure
                            else
                            {
                              sprintf( (char *)tempBffr2, "Pressure TASKING ERROR!" );
                            } // EndElse (Status == HAL_OK) RoadBrd_Baro_ReadPressure
                          } // Endif (Status == HAL_OK) RoadBrd_Humidity_ReadHumidity
                          else
                          {
                            sprintf( (char *)tempBffr2, "Humidity TASKING ERROR!" );
                          } // EndElse (Status == HAL_OK) RoadBrd_Humidity_ReadHumidity
                        } // Endif (Status == HAL_OK) RoadBrd_ReadTemp
                        else
                        {
                          sprintf( (char *)tempBffr2, "AMBIENT TEMPERATURE TASKING ERROR!" );
                        } // EndElse (Status == HAL_OK) RoadBrd_ReadTemp
                      } // Endif (Status == HAL_OK) RoadBrd_CoolEye_ReadValues
                      else
                      {
                        sprintf( (char *)tempBffr2, "GRID EYE/COOL EYE TASKING ERROR!" );
                      } // EndElse (Status == HAL_OK) RoadBrd_CoolEye_ReadValues
                    }
                    else
                    {
                      switch( tempBffr[2] )
                      {
                        //------------------ TCS Command: Calibration Set Command
                      case 'S':
                        // Step 1. Validate format.
                        if(tempBffr[3]!=':')
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          strcpy( (char *)tempBffr2, "TCS SYNTAX ERROR: Not correct format.\r\n");
                        } // Endif (tempBffr[3]!=':')
                        else
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          // 2. Verify if remaining string is digits
                          if (Size <= 4)
                          {
                            strcpy( (char *)tempBffr2, "TCS SYNTAX ERROR: Bad Parameter.\r\n");
                          } // EndIf (Size > 4)
                          else
                          {
                            // 3. Grab remaining string and convert to integer.
                            tempPstr = &tempBffr[4];
                            strcpy(tempstr, tempPstr);
                            // Time to parse and test remaining string
                            Scale = 0.0;
                            Offset = 0.0;
                            if (sscanf (tempstr, "%s %f %f", uuid, &Scale, &Offset) == 3)
                            {
                              sprintf( (char *)tempBffr2, "Parms: %s, %f, %f.\r\n", uuid, Scale, Offset );
                              // OK, We have 3 good parameters... NOW Need to determine if UUID is good.
                              if (strncmp((char *)uuid,"0002",4) == 0) // Shnt_Vltg
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_SHNT_VLTG, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Shnt_Vltg Set COMPLETE.\r\n" );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0004",4) == 0) // Current
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_CURRENT, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Current Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0006",4) == 0) // Power
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_POWER, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Power Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0008",4) == 0) // Voltage
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_VOLTAGE, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Voltage Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"000A",4) == 0) // TempC
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_TEMPC, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "TempC Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"000B",4) == 0) // TempF
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_TEMPF, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "TempF Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0011",4) == 0) // Pressure
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_PRESSURE, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Pressure Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0030",4) == 0) // Humidity
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_HUMIDITY, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Humidity Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0032",4) == 0) // Hum_TempC
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_HUM_TEMPC, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Hum_TempC Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0033",4) == 0) // Hum_TempF
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_HUM_TEMPF, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Hum_TempF Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"000D",4) == 0) // RGB_Red
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_RGB_RED, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RGB_Red Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"000E",4) == 0) // RGB_Green
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_RGB_GREEN, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RGB_Green Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"000F",4) == 0) // RGB_Blue
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_RGB_BLUE, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RGB_Blue Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0017",4) == 0) // Therm_C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_THERM_C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "Therm_C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0019",4) == 0) // RoadT_1C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_1C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RoadT_1C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"001B",4) == 0) // RoadT_2C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_2C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RoadT_2C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"001D",4) == 0) // RoadT_3C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_3C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RoadT_3C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"001F",4) == 0) // RoadT_4C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_4C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RoadT_4C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0021",4) == 0) // RoadT_5C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_5C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RoadT_5C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0023",4) == 0) // RoadT_6C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_6C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RoadT_6C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0025",4) == 0) // RoadT_7C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_7C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RoadT_7C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else if (strncmp((char *)uuid,"0027",4) == 0) // RoadT_8C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_8C, Offset, Scale);
                                if (Status == HAL_OK)
                                  sprintf( (char *)tempBffr2, "RoadT_8C Set COMPLETE." );
                                else
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                              }
                              else
                              {
                                sprintf( (char *)tempBffr2, "TCS SYNTAX ERROR: Bad UUID.\r\n" );
                              }
                              Status = HAL_OK;
                            }
                            else
                            {
                              strcpy( (char *)tempBffr2, "TCS SYNTAX ERROR: Wrong Number of Parameters.\r\n");
                            }
                          } // EndElse (flag == 0)
                        } // EndElse (tempBffr[3]!=':')
                        break;
                        //------------------ TCR Command: Calibration Read Command
                      case 'R':
                        // Build Read Calibration Dump Part I....
                        // Is this a BLE Operation?
                        if ( BLE_Flag )
                        {
                          // Yes...Build and Send BLE Response NOW.
                          strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                          BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                        }
                        
                        // Send string to UART..
                        sprintf( (char *)tempBffr2, "CALIBRATION DATA\r\nDate: %s\r\n",  RoadBrd_CAL_GetTimeString());
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                            return Status;
                        // Build Read Calibration Dump Part II....
                        // Send string to UART..
                        sprintf( (char *)tempBffr2, "Name		        UUID		Slope		Offset\r\n" );
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                            return Status;
                        // NOW, Build Data String..
                        for (x=0; x<CAL_LAST_VALUE; x++)
                        {
                          // Build String
                          switch(x)
                          {
                            case CAL_SHNT_VLTG: //CAL_SHNT_VLTG Values
                              sprintf( (char *)tempBffr2, "%s	0002		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_CURRENT: //CAL_CURRENT Values
                              sprintf( (char *)tempBffr2, "%s	        0004		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_POWER: //CAL_POWER Values
                              sprintf( (char *)tempBffr2, "%s	0006		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_VOLTAGE: //CAL_VOLTAGE Values
                              sprintf( (char *)tempBffr2, "%s	        0008		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_TEMPC: //CAL_TEMPC Values
                              sprintf( (char *)tempBffr2, "%s	000A		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_TEMPF: //CAL_TEMPF Values
                              sprintf( (char *)tempBffr2, "%s	000B		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_PRESSURE: //CAL_PRESSURE Values
                              sprintf( (char *)tempBffr2, "%s	0011		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_HUMIDITY: //CAL_HUMIDITY Values
                              sprintf( (char *)tempBffr2, "%s	0030		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_HUM_TEMPC: //CAL_HUM_TEMPC Values
                              sprintf( (char *)tempBffr2, "%s	0032		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_HUM_TEMPF: //CAL_HUM_TEMPF Values
                              sprintf( (char *)tempBffr2, "%s	0033		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_RGB_RED: //CAL_RGB_RED Values
                              sprintf( (char *)tempBffr2, "%s	        000D		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_RGB_GREEN: //CAL_RGB_GREEN Values
                              sprintf( (char *)tempBffr2, "%s	000E		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_RGB_BLUE: //CAL_RGB_BLUE Values
                              sprintf( (char *)tempBffr2, "%s	000F		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_THERM_C: //CAL_THERM_C Values
                              sprintf( (char *)tempBffr2, "%s	        0017		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_1C: //CAL_ROADT_1C Values
                              sprintf( (char *)tempBffr2, "%s	0019		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_2C: //CAL_ROADT_2C Values
                              sprintf( (char *)tempBffr2, "%s	001B		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_3C: //CAL_ROADT_3C Values
                              sprintf( (char *)tempBffr2, "%s	001D		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_4C: //CAL_ROADT_4C Values
                              sprintf( (char *)tempBffr2, "%s	001F		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_5C: //CAL_ROADT_5C Values
                              sprintf( (char *)tempBffr2, "%s	0021		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_6C: //CAL_ROADT_6C Values
                              sprintf( (char *)tempBffr2, "%s	0023		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_7C: //CAL_ROADT_7C Values
                              sprintf( (char *)tempBffr2, "%s	0025		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_8C: //CAL_ROADT_8C Values
                              sprintf( (char *)tempBffr2, "%s	0027		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                          } // EndSwitch(x)
                          // Now Print String.
#ifdef NUCLEO
                          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                          Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                          if (Status != HAL_OK)
                            return Status;
                        } // EndFor(x=0; x<CAL_LAST_VALUE; x++)
                        sprintf( (char *)tempBffr2, "\r\n     COMPLETE.\r\n" );
                        break;
                        //------------------ TCT Command: Calibration Set Time Command
                      case 'T':
                        // Step 1. Validate format.
                        if(tempBffr[3]!=':')
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          strcpy( (char *)tempBffr2, "TCT SYNTAX ERROR: Not correct format.\r\n");
                        } // Endif (tempBffr[3]!=':')
                        else
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          // 2. Verify if remaining string is digits
                          if (Size <= 4)
                          {
                            strcpy( (char *)tempBffr2, "TCT SYNTAX ERROR: Bad Parameter.\r\n");
                          } // EndIf (Size > 4)
                          else
                          {
                            // 3. Grab remaining string and Save it.
                            tempPstr = &tempBffr[4];
                            strcpy(tempstr, tempPstr);
                            // NOW...Save it.
                            Status = RoadBrd_CAL_Set_TimeString( (uint8_t *)tempPstr );
                            if (Status != HAL_OK)
                              return Status;
                            sprintf( (char *)tempBffr2, "\r\n     COMPLETE.\r\n" );
                          } // EndElse (Size > 4)
                        } // EndElse (tempBffr[3]!=':')
                        break;
                        //------------------ TCI Command: Calibration Initialize Cal Table(Reset)
                      case 'I':
                        Status = RoadBrd_CAL_InitializeFrmFlash();
                        // Is this a BLE Operation?
                        if ( BLE_Flag )
                        {
                          // Yes...Build and Send BLE Response NOW.
                          strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                          BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                        }
                        
                        if (Status != HAL_OK)
                          return Status;
                        sprintf( (char *)tempBffr2, "\r\n     COMPLETE.\r\n" ); 
                        break;
                      } //EndSwitch
                    } //EndElse (Size == 2)*/
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Dump Driver State.
                  case 'D':
                    // Read Driver Status
                    DriverStatus = Get_DriverStatus();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>ST_DRIVER:%04x</STATUS>", DriverStatus );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    sprintf( (char *)tempBffr2, "Driver Status: %04x\r\n", DriverStatus );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  RESET Micro.
                  case 'R':
                    // RESET
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>ST_RESET_ACK</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      delay_100msec(1);;           // Wait 100ms
                    }
                    strcpy( (char *)tempBffr2, "TR CMD: Reset Micro NOT IMPLEMENTED\r\n");
                    //HAL_NVIC_SystemReset();
                    SkyPack_Reset( ERROR_NULL );
                    sprintf( (char *)tempBffr2, "RESET CALLED BUT NO RESPONSE!!\r\n" );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Key Flash Variable Commands.
                  case 'K':
                    // Key Flash Variable Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    Status = HAL_OK;
                    if (Size < 4)
                    {
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_TK_SYNTAX</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      strcpy( (char *)tempBffr2, "TK SYNTAX ERROR: Not correct format.\r\n");
                    }
                    else
                    {
                      switch( tempBffr[2] )
                      {
//------------------
                        case 'S':
                          //Key Flash Variable Set Command.
                          switch( tempBffr[3] )
                          {
//------------------
                            case 'S':
                              //Key Flash Variable Set Sensor Sample Rate Command.
                              // Step 1. Validate format.
                              if(tempBffr[4]!=':')
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSS_SYNTAX</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Not correct format.\r\n");
                              } // Endif (tempBffr[4]!=':')
                              else
                              {
                                // 2. Verify if remaining string is digits
                                if (Size > 5)
                                {
                                  flag = 1;
                                  for (x=5; x< Size; x++)
                                  {
                                    if (isdigit(tempBffr[x]) == 0)
                                      flag = 0;
                                  }
                                } // EndIf (Size > 5)
                                else
                                  flag = 0;
                                if (flag == 0)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSS_BADPARAM</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Bad Parameter.\r\n");
                                }
                                else
                                {
                                  // 3. Grab remaining string and convert to integer.
                                  tempPstr = &tempBffr[5];
                                  strcpy(tempstr, tempPstr);
                                  new_value = atoi( tempstr );
                                  if((new_value > 9999) ||
                                     (new_value < 0))
                                  {
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSS_BADPARAM</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Bad Parameter.\r\n");
                                  }
                                  else
                                  {
                                    // Time to set new Road Sound Sample Rate.
#ifdef STM32L151CBT6
                                    SkyBrd_Set_SnsrTickCnt( new_value );
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>ST_TKSS_ACK</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
#endif
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "COMPLETE" );
                                  } // EndElse ((new_value > 9999) || (new_value < 0))
                                } // EndElse (flag == 0)
                              } // EndElse (tempBffr[4]!=':')
                              break;
//------------------
                            case 'T':
                              //Key Flash Variable Set TACK Limit(Multiple of Road Sound Throttles).
                              // Step 1. Validate format.
                              if(tempBffr[4]!=':')
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TKST_SYNTAX</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                strcpy( (char *)tempBffr2, "TKST SYNTAX ERROR: Not correct format.\r\n");
                              } // Endif (tempBffr[4]!=':')
                              else
                              {
                                // 2. Verify if remaining string is digits
                                if (Size > 5)
                                {
                                  flag = 1;
                                  for (x=5; x< Size; x++)
                                  {
                                    if (isdigit(tempBffr[x]) == 0)
                                      flag = 0;
                                  }
                                } // EndIf (Size > 5)
                                else
                                  flag = 0;
                                if (flag == 0)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>CMD_TKST_BADPARAM</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  strcpy( (char *)tempBffr2, "TKST SYNTAX ERROR: Bad Parameter.\r\n");
                                }
                                else
                                {
                                  // 3. Grab remaining string and convert to integer.
                                  tempPstr = &tempBffr[5];
                                  strcpy(tempstr, tempPstr);
                                  new_value = atoi( tempstr );
                                  if((new_value > 9999) ||
                                     (new_value < 0))
                                  {
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>CMD_TKST_BADPARAM</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Bad Parameter.\r\n");
                                  }
                                  else
                                  {
                                    // Time to set new TACK Limit.
                                    SkyBrd_Set_TackLimit( new_value );
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>ST_TKST_ACK</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "COMPLETE" );
                                  } // EndElse ((new_value > 9999) || (new_value < 0))
                                } // EndElse (flag == 0)
                              } // EndElse (tempBffr[4]!=':')
                              break;
                            case 'B':
                              //Key Flash Variable Set Boot Delay(Seconds).
                              // Step 1. Validate format.
                              if(tempBffr[4]!=':')
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSB_SYNTAX</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                strcpy( (char *)tempBffr2, "TKSB SYNTAX ERROR: Not correct format.\r\n");
                              } // Endif (tempBffr[4]!=':')
                              else
                              {
                                // 2. Verify if remaining string is digits
                                if (Size > 5)
                                {
                                  flag = 1;
                                  for (x=5; x< Size; x++)
                                  {
                                    if (isdigit(tempBffr[x]) == 0)
                                      flag = 0;
                                  }
                                } // EndIf (Size > 5)
                                else
                                  flag = 0;
                                if (flag == 0)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSB_BADPARAM</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  strcpy( (char *)tempBffr2, "TKSB SYNTAX ERROR: Bad Parameter.\r\n");
                                }
                                else
                                {
                                  // 3. Grab remaining string and convert to integer.
                                  tempPstr = &tempBffr[5];
                                  strcpy(tempstr, tempPstr);
                                  new_value = atoi( tempstr );
                                  if((new_value > 999) ||
                                     (new_value < 0))
                                  {
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSB_BADPARAM</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    strcpy( (char *)tempBffr2, "TKSB SYNTAX ERROR: Bad Parameter.\r\n");
                                  }
                                  else
                                  {
                                    // Time to set new Boot Delay.
                                    SkyBrd_Set_BootDelay( new_value );
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>ST_TKSB_ACK</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "COMPLETE" );
                                  } // EndElse ((new_value > 999) || (new_value < 0))
                                } // EndElse (flag == 0)
                              } // EndElse (tempBffr[4]!=':')
                              break;
                            default:
                              strcpy( (char *)tempBffr2, "TKS ERROR: Not a legal command.\r\n");
                              break;
                          } // EndSwitch ( tempBffr[3] )
                          break;
//------------------
                        case 'R':
                          //Key Flash Variable Read Command
                          switch( tempBffr[3] )
                          {
//------------------
                            case 'S':
                              //Key Flash Variable Read Sensor Sample Rate Command.
#ifdef STM32L151CBT6
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>ST_TKRS:%3.1f</STATUS>", ((float)SkyBrd_GetSampleTime()/10));
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "Sensor Sample Rate: %3.1f Seconds.\r\n\r\n> ", ((float)SkyBrd_GetSampleTime()/10));
#else
                              sprintf( (char *)tempBffr2, "Sensor Sample Rate: (NOT SUPPORTED) Seconds.\r\n\r\n> " );
#endif
                              break;
//------------------
                            case 'T':
                              //Key Flash Variable Read TACK Limit(Multiple of Road Sound Throttles).
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>ST_TKRT:%d</STATUS>", SkyBrd_Get_TackLimit());
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "TACK Limit: %d.\r\n\r\n> ", SkyBrd_Get_TackLimit());
                              break;
//------------------
                            case 'B':
                              //Key Flash Variable Read Boot Delay.(Seconds).
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>ST_TKRB:%d</STATUS>", SkyBrd_Get_BootDelay());
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "Boot Delay: %d Seconds.\r\n\r\n> ", SkyBrd_Get_BootDelay());
                              break;
                            default:
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>CMD_TKR_SYNTAX</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              strcpy( (char *)tempBffr2, "TKR ERROR: Not a legal command.\r\n");
                              break;
                          } // EndSwitch ( tempBffr[3] )
                          break;
                        default:
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TK_SYNTAX</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }                          strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                          break;
                      } //EndSwitch ( tempBffr[2] )
                    } //EndElse (Size < 3)
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Units Enable/Disable Commands.
                  case 'U':
                    // Key Flash Variable Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    Status = HAL_OK;
                    if (Size < 3)
                    {
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_TU_SYNTAX</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      strcpy( (char *)tempBffr2, "TU SYNTAX ERROR: Not correct format.\r\n");
                    }
                    else
                    {
                      switch( tempBffr[2] )
                      {
//------------------
                        case 'E':
                          //Units Enable Command.
                          sprintf( (char *)tempBffr2, "Units XML State CHANGED: ENABLED\r\n\r\n> ");
#ifdef STM32L151CBT6
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>ST_TUE_ACK</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          Status = SkyBrd_Set_UnitsFlag( true );
#endif
                          break;
//------------------
                        case 'D':
                          //Units Disable Command
                          sprintf( (char *)tempBffr2, "Units XML State CHANGED: DISABLED\r\n\r\n> ");
#ifdef STM32L151CBT6
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>ST_TUD_ACK</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          Status = SkyBrd_Set_UnitsFlag( false );
#endif
                          break;
                        default:
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TU_SYNTAX</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          strcpy( (char *)tempBffr2, "TU SYNTAX ERROR: Not a legal command.\r\n");
                          break;
                      } //EndSwitch ( tempBffr[2] )
                    } //EndElse (Size < 3)
                    break;
 //++++++++++++++++++++++++++++++++++++++++++  Reset Flash Frame Variables(Factory).
                  case 'F':
                    // Reset Flash Frame Variables.
                    SkyBrd_WWDG_InitializeFrmFlash();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>TF_ACK</STATUS>" );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    sprintf( (char *)tempBffr2, "Flash Frame Variables Reset to Factory Values.\r\n" );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Lock Code to allow stable Program of BLE Module.
                  case 'L':
                    // Read Driver Status
                    DriverStatus = Get_DriverStatus();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>TL_ERROR</STATUS>" );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    else
                    {
                      sprintf( (char *)tempBffr2, "Code Locked for Programming!\r\n\r\n" );
                      Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                      if (Status != HAL_OK)
                        return Status;
                      sprintf( (char *)tempBffr2, "   HARD RESET NEEDED TO EXIT MODE\r\n" );
                      Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                      if (Status != HAL_OK)
                        return Status;
                      // Start Hard Loop
                      for (;;)
                      {
                      }
                    }
                    
                    sprintf( (char *)tempBffr2, "Driver Status: %04x\r\n", DriverStatus );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Unknown Command.
                  default:
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    // ERROR if we get here.. 
                    strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                    break;
                 
                } //EndSwitch ( tempBffr[1] )
              } //EndElse (Size <= 1)
              break;
            default:
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              
              // ERROR if we get here.. 
              strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
              break;
            } // EndSwitch
            

            // Test last I2C Status to determine next msg.
            switch( Status )
            {
              case HAL_OK:
                break;
              case HAL_ERROR:
                // Determine what kind of error.
/*                Err_code = RoadBrd_I2C_GetError();
                switch( Err_code )
                {
                  case HAL_I2C_ERROR_BERR:
                    strcpy( (char *)tempBffr2, "I2C ERROR: BERR: Bus Error reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_ARLO:
                    strcpy( (char *)tempBffr2, "I2C ERROR: ARLO: Arbitration lost reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_AF:
                    strcpy( (char *)tempBffr2, "I2C ERROR: AF: Acknowledge failure reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_OVR:
                    strcpy( (char *)tempBffr2, "I2C ERROR: OVR: Overrun/Underrun reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_DMA:
                    strcpy( (char *)tempBffr2, "I2C ERROR: DMA: DMA Error reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_TIMEOUT:
                    strcpy( (char *)tempBffr2, "I2C ERROR: TIMEOUT: Timeout Error reported.\r\n");
                    break;
                  default: 
                    strcpy( (char *)tempBffr2, "I2C ERROR: Unknown Error reported.\r\n");
                    break;
                 
                }
                // Re-Initialize I2C....It has been corrupted.
                MX_I2C1_Reset(); */
                strcpy( (char *)tempBffr2, "Sensor ERROR: Sensor Read FAILED.\r\n");
                break;
              case HAL_BUSY:
                strcpy( (char *)tempBffr2, "Sensor ERROR: Sensor Read BUSY error.\r\n");
                // Re-Initialize I2C....It has been corrupted.
//                MX_I2C1_Reset();
                break;
              case HAL_TIMEOUT:
                strcpy( (char *)tempBffr2, "Sensor ERROR: Sensor Read TIMEOUT.\r\n");
                // Re-Initialize I2C....It has been corrupted.
//                MX_I2C1_Reset();
                break;
              default:  
                strcpy( (char *)tempBffr2, "Sensor ERROR: Sensor Read unnown error.\r\n");
                // Re-Initialize I2C....It has been corrupted.
//                MX_I2C1_Reset();
                break;
            } 
            // Send string to UART..
            Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
            if (Status != HAL_OK)
              return Status;

            // Send Prompt to UART..
            strcpy( (char *)tempBffr2, "\r\n\r\n> ");
            Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
            if (Status != HAL_OK)
              return Status;
    } // EndElse ( Bypass )
  return Status;
}

/**
  * @brief  This function tests the passed string to make sure it is Hex format.
  * @param  char *ptr: Ptr to string to be tested. NULL terminated
  * @retval int: 1: String is HEX.
  *              0:   String is not HEX.
  */
int isHexNum(char *ptr)
{
  int Size, x, test1;
  
  Size = strlen(ptr);
  test1 = 0;
  for (x=0; x<Size; x++)
  {
    if (((ptr[x] <= '9') && (ptr[x] >= '0')) ||
        ((ptr[x] <= 'F') && (ptr[x] >= 'A')))
      test1 = 0;
    else
      test1 = 1;
    if (test1 == 1)
      return 0;
  }
  return 1;
}

/**
  * @brief  This function converts the passed Hex String to an Integer value.
  * @param  char *ptr: Ptr to string to be converted. NULL terminated
  * @retval int: -1: Error in String
  *              Value converted returned.
  */
int hatoi( char *ptr )
{
  int Size, x;
  int Value = 0;
  int FinalValue = 0;
  
  Size = strlen(ptr);
  for (x=0; x<Size; x++)
  {
    if ((ptr[x] <= '9') && (ptr[x] >= '0'))
      Value = ptr[x] - '0';
    else if((ptr[x] <= 'F') && (ptr[x] >= 'A'))
      Value = ptr[x] - 'A' + 10;
    else
      return -1;
    FinalValue = FinalValue*16 + Value;
  }
  return FinalValue;
}


bool Tst_Bypass( void)
{
  return Bypass;
}

/* Read the IMU sensor data
 * - Accelerometer data is in 1/1000 g
 * - Gyro data is in 1/10 dps
 * - Magnetometer data is in mGauss
 * All scaling done according to typical sensitivity values in datasheet */

HAL_StatusTypeDef ReadIMUSensorsLocal(void)
{
  int i;
  union u3DVector vector;
  
  uint8_t id = 0;
  LSM6DS3_Read_XG_ID(&id);
  
  /* Read the accelerometer data */
  if (LSM6DS3_X_GetAxesRaw(vector.indexed) == HAL_OK)
  {
    /* Scale the values so they are in 1/1000 g when we run the sensor
     * at 4 g full scale (0.122 mg/LSB) */
    for (i=0; i<3; i++)
    {
      imu.accel.indexed[i] = (int16_t)
                        (((int32_t)(vector.indexed[i]) * 4000) / 32786);
    }
  }
  else
    return HAL_ERROR;
  
  /* Read the gyro data */
  if (LSM6DS3_G_GetAxesRaw(vector.indexed) == HAL_OK)
  {
    /* Scale the values so they are in 1/10 dps when we run the sensor
     * at 500 dps full scale (17.5 mdps/LSB) */
    for (i=0; i<3; i++)
    {
      imu.gyro.indexed[i] = (int16_t)
                        (((int32_t)(vector.indexed[i]) * 5000) / 28571);
    }
  }
  else
    return HAL_ERROR;
  
  /* Read the magnetometer data */
  if (LIS3MDL_M_GetAxesRaw(vector.indexed) == HAL_OK)
  {
    /* Scale the values so they are in mgauss when we run the sensor
     * at 4 gauss full scale (6842 LSB/gauss) */
    for (i=0; i<3; i++)
    {
      imu.mag.indexed[i] = (int16_t)
                        (((int32_t)(vector.indexed[i]) * 4000) / (6842 * 4));
    }
  }
  else
    return HAL_ERROR;
  
  return HAL_OK;
}

/*void sleep(void) {
//    TimMasterHandle.Instance = TIM5;
 
    // Disable HAL tick interrupt
//    __HAL_TIM_DISABLE_IT(&TimMasterHandle, TIM_IT_CC2);
 
    // Request to enter SLEEP mode
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
 
    // Enable HAL tick interrupt
//    __HAL_TIM_ENABLE_IT(&TimMasterHandle, TIM_IT_CC2);
}*/
 
/*void deepsleep(void) {
    // Request to enter STOP mode with regulator in low power mode
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
 
    // After wake-up from STOP reconfigure the PLL
    //SetSysClock();
    SystemClock_Config();
}*/

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
