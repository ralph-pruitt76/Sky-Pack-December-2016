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
#include "Calibration.h"
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
    char uuid[10];
    float Scale, Offset;
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
                  sprintf( (char *)tempBffr2, "<STATUS>X:%05d/%05.2f//Y:%05d/%05.2f//Z:%05d/%05.2f</STATUS>", 
                          imu.accel.named.x, 
                          SkyPack_CAL_ScaleValue( CAL_IMU_X, imu.accel.named.x),
                          imu.accel.named.y, 
                          SkyPack_CAL_ScaleValue( CAL_IMU_Y, imu.accel.named.y),
                          imu.accel.named.z, 
                          SkyPack_CAL_ScaleValue( CAL_IMU_Z, imu.accel.named.z) );
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Dump Accelerometer Data...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     X DATA: %05d/%05.2f (0.001g)\r\n", imu.accel.named.x, SkyPack_CAL_ScaleValue( CAL_IMU_X, imu.accel.named.x) );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Y DATA: %05d/%05.2f (0.001g)\r\n", imu.accel.named.y, SkyPack_CAL_ScaleValue( CAL_IMU_Y, imu.accel.named.y) );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Z DATA: %05d/%05.2f (0.001g)\r\n", imu.accel.named.z, SkyPack_CAL_ScaleValue( CAL_IMU_Z, imu.accel.named.z) );
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
                  sprintf( (char *)tempBffr2, "<STATUS>X:%05d/%05.1f//Y:%05d/%05.1f//Z:%05d/%05.1f</STATUS>", 
                          imu.gyro.named.x, 
                          SkyPack_CAL_ScaleValue( CAL_GYRO_X, imu.gyro.named.x),
                          imu.gyro.named.y, 
                          SkyPack_CAL_ScaleValue( CAL_GYRO_Y, imu.gyro.named.y),
                          imu.gyro.named.z, 
                          SkyPack_CAL_ScaleValue( CAL_GYRO_Z, imu.gyro.named.z) );
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Dump Gyro Data...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     X DATA: %05d/%05.1f (0.1dps)\r\n", imu.gyro.named.x, SkyPack_CAL_ScaleValue( CAL_GYRO_X, imu.gyro.named.x) );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Y DATA: %05d/%05.1f (0.1dps)\r\n", imu.gyro.named.y, SkyPack_CAL_ScaleValue( CAL_GYRO_Y, imu.gyro.named.y) );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Z DATA: %05d/%05.1f (0.1dps)\r\n", imu.gyro.named.z, SkyPack_CAL_ScaleValue( CAL_GYRO_Z, imu.gyro.named.z) );
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
                  sprintf( (char *)tempBffr2, "<STATUS>X:%05d/%05.3f//Y:%05d/%05.3f//Z:%05d/%05.3f</STATUS>", 
                          imu.mag.named.x, 
                          SkyPack_CAL_ScaleValue( CAL_MAG_X, imu.mag.named.x),
                          imu.mag.named.y, 
                          SkyPack_CAL_ScaleValue( CAL_MAG_Y, imu.mag.named.y),
                          imu.mag.named.z, 
                          SkyPack_CAL_ScaleValue( CAL_MAG_Z, imu.mag.named.z) );
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Dump Magnetometer Data...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     X DATA: %05d/%05.3f (mgauss)\r\n", imu.mag.named.x, SkyPack_CAL_ScaleValue( CAL_MAG_X, imu.mag.named.x) );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Y DATA: %05d/%05.3f (mgauss)\r\n", imu.mag.named.y, SkyPack_CAL_ScaleValue( CAL_MAG_Y, imu.mag.named.y) );
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                sprintf( (char *)tempBffr2, "     Z DATA: %05d/%05.3f (mgauss)\r\n", imu.mag.named.z, SkyPack_CAL_ScaleValue( CAL_MAG_Z, imu.mag.named.z) );
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
                  sprintf( (char *)tempBffr2, "<STATUS>PRESSURE: %5.3f/%5.3f</STATUS>", value, SkyPack_CAL_ScaleValue( CAL_PRESSURE, value) );
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Pressure Sensor...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                // Now Display.
                sprintf( (char *)tempBffr2, "     Pressure: %5.3f/%5.3fmbar\r\n", value, SkyPack_CAL_ScaleValue( CAL_PRESSURE, value));
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
                  sprintf( (char *)tempBffr2, "<STATUS>TEMPC:%3.1fC/%3.1fC</STATUS>", 
                          value, 
                          SkyPack_CAL_ScaleValue( CAL_TEMPC, value) );
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Send string to UART..
                strcpy( (char *)tempBffr2, "Temperature Sensor...\r\n");
                Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                if (Status != HAL_OK)
                  return Status;
                // Now calculate Celcius and Farenheit Temp.
                sprintf( (char *)tempBffr2, "     Temperature: %3.1fC/%3.1fC\r\n", value, SkyPack_CAL_ScaleValue( CAL_TEMPC, value));
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
                  sprintf( (char *)tempBffr2, "<STATUS>IRRAD:%08d/%08.1f(100lx)</STATUS>", 
                          irradiance, 
                          SkyPack_CAL_ScaleValue( CAL_IRRADIANCE, irradiance) );
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
              // Send string to UART..
              strcpy( (char *)tempBffr2, "Irradiance Light Sensor...\r\n");
              Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
              if (Status != HAL_OK)
                return Status;
              // Now calculate.
              sprintf( (char *)tempBffr2, "     Irradiance: %08d/%08.1f(100lx)\r\n", irradiance, SkyPack_CAL_ScaleValue( CAL_IRRADIANCE, irradiance));
              break;    
//**************************************************************************************************
            case 'C':
              // Read Cap Sense Event Frequency. 
              cap.event_freq = GetCapSenseEventFreq();
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                sprintf( (char *)tempBffr2, "<STATUS>CAP_SNS:%03.2f/%03.2f(Hz/100)</STATUS>", 
                        (float)(cap.event_freq/100), 
                        (float)(SkyPack_CAL_ScaleValue( CAL_CAP_SENSE, cap.event_freq)) );
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              // Send string to UART..
              strcpy( (char *)tempBffr2, "Cap Sense Event Frequency...\r\n");
              Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
              if (Status != HAL_OK)
                return Status;
              // Now calculate.
              sprintf( (char *)tempBffr2, "     Frequency: %03.2f/%03.2f(Hz/100)\r\n", (float)(cap.event_freq/100), (float)(SkyPack_CAL_ScaleValue( CAL_CAP_SENSE, cap.event_freq)));
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
                sprintf( (char *)tempBffr2, "<STATUS>SW_FREQ:%d/%fIDX//%d/%fLVL</STATUS>", 
                        cap.swept_idx, 
                        SkyPack_CAL_ScaleValue( CAL_SWPT_FREQ, cap.swept_idx), 
                        cap.swept_level, 
                        SkyPack_CAL_ScaleValue( CAL_SWPT_LEVL, cap.swept_level) );
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
              sprintf( (char *)tempBffr2, "          highest output level: %d/%f\r\n", cap.swept_idx, SkyPack_CAL_ScaleValue( CAL_SWPT_FREQ, cap.swept_idx));
              Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
              if (Status != HAL_OK)
                return Status;

              sprintf( (char *)tempBffr2, "     Swept Frequency Sensor highest level: %d/%fHz\r\n", cap.swept_level, SkyPack_CAL_ScaleValue( CAL_SWPT_LEVL, cap.swept_level));
 
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
                    if (Size == 2)
                    {
                      //------------------ TC Command: Dump Calibration Settings.      
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      sprintf( (char *)tempBffr2, "TC CMD: No TC ONLY CMD for Sky Pack.\r\n" );
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
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TCS_SYNTAX</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          strcpy( (char *)tempBffr2, "TCS SYNTAX ERROR: Not correct format.\r\n");
                        } // Endif (tempBffr[3]!=':')
                        else
                        {
                          // 2. Verify if remaining string is digits
                          if (Size <= 4)
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_TCS_BADPARAM</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
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
                              // Test for 	IMU..Acceleration X...Scale and Offset...B6B6X
                              if (strncmp((char *)uuid,"B6B6X",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_IMU_X, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_ACCL_X</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Acceleration X Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_ACCL_X_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	IMU..Acceleration Y...Scale and Offset...B6B6Y
                              else if (strncmp((char *)uuid,"B6B6Y",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_IMU_Y, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_ACCL_Y</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Acceleration Y Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_ACCL_Y_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	IMU..Acceleration Z...Scale and Offset...B6B6Z
                              else if (strncmp((char *)uuid,"B6B6Z",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_IMU_Z, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_ACCL_Z</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Acceleration Z Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_ACCL_Z_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	IMU..Gyro X...Scale and Offset...B9B9X
                              else if (strncmp((char *)uuid,"B9B9X",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_GYRO_X, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_GYRO_X</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Gyro X Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_GYRO_X_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	IMU..Gyro Y...Scale and Offset...B9B9Y
                              else if (strncmp((char *)uuid,"B9B9Y",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_GYRO_Y, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_GYRO_Y</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Gyro Y Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_GYRO_Y_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	IMU..Gyro Z...Scale and Offset...B9B9Z
                              else if (strncmp((char *)uuid,"B9B9Z",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_GYRO_Z, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_GYRO_Z</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Gyro Z Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_GYRO_Z_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	IMU..Magnatometer X...Scale and Offset...B7B7X
                              else if (strncmp((char *)uuid,"B7B7X",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_MAG_X, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_MAG_X</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Magnatometer X Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_MAG_X_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	IMU..Magnatometer Y...Scale and Offset...B7B7Y
                              else if (strncmp((char *)uuid,"B7B7Y",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_MAG_Y, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_MAG_Y</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Magnatometer Y Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_MAG_Y_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	IMU..Magnatometer Z...Scale and Offset...B7B7Z
                              else if (strncmp((char *)uuid,"B7B7Z",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_MAG_Z, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_MAG_Z</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "IMU..Magnatometer Z Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_IMU_MAG_Z_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	Temperature...Scale and Offset...B8B8
                              else if (strncmp((char *)uuid,"B8B8",4) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_TEMPC, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_TEMPC</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Temperature Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_TEMPC_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	Pressure...Scale and Offset...BEBE
                              else if (strncmp((char *)uuid,"BEBE",4) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_PRESSURE, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_PRESSURE</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Pressure Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_PRESSURE_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	Irradiance...Scale and Offset...BBBB
                              else if (strncmp((char *)uuid,"BBBB",4) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_IRRADIANCE, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_IRRADIANCE</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Irradiance Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_IRRADIANCE_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	cap sense event frequency...Scale and Offset...BCBC
                              else if (strncmp((char *)uuid,"BCBC",4) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_CAP_SENSE, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_CAP_SENSE</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Cap Sense Event Frequency Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_CAP_SENSE_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	swept frequency event index...Scale and Offset...BDBDI
                              else if (strncmp((char *)uuid,"BDBDI",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_SWPT_FREQ, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_SWPT_FREQ_X</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Swept Frequency Event Index Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_SWPT_FREQ_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              // Test for	swept frequency event level...Scale and Offset...BDBDL
                              else if (strncmp((char *)uuid,"BDBDL",5) == 0)
                              {
                                Status = SkyPack_CAL_Set_CalItem( CAL_SWPT_LEVL, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_SWPT_LEVL_X</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Swept Frequency Event Level Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_CAL_SWPT_LEVL_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TCS_BADUUID</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                sprintf( (char *)tempBffr2, "TCS SYNTAX ERROR: Bad UUID.\r\n" );
                              }
                              Status = HAL_OK;
                            }
                            else
                            {
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>CMD_TCS_BADPARAM</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              
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
                        sprintf( (char *)tempBffr2, "CALIBRATION DATA\r\nDate: %s\r\n",  SkyPack_CAL_GetTimeString());
                        Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                        if (Status != HAL_OK)
                            return Status;
                        // Build Read Calibration Dump Part II....
                        // Send string to UART..
                        sprintf( (char *)tempBffr2, "Name		        UUID		Slope		Offset\r\n" );
                        Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
                        if (Status != HAL_OK)
                            return Status;
                        // NOW, Build Data String..
                        for (x=0; x<CAL_LAST_VALUE; x++)
                        {
                          // Build String
                          switch(x)
                          {
                            case CAL_IMU_X: //CAL_IMU_X / Accelerometer Characteristic X Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B6B6X//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B6B6X		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_IMU_Y: //CAL_IMU_Y / Accelerometer Characteristic Y Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B6B6Y//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B6B6Y		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_IMU_Z: //CAL_IMU_Z / Accelerometer Characteristic Z Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B6B6Z//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B6B6Z		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_GYRO_X: //CAL_GYRO_X / Gyro Characteristic X Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B9B9X//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B9B9X		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_GYRO_Y: //CAL_GYRO_Y / Gyro Characteristic Y Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B9B9Y//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B9B9Y		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_GYRO_Z: //CAL_GYRO_Z / Gyro Characteristic Z Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B9B9Z//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B9B9Z		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_MAG_X: //CAL_MAG_X / Magnetometer Characteristic X Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B7B7X//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B7B7X		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_MAG_Y: //CAL_MAG_Y / Magnetometer Characteristic Y Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B7B7Y//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B7B7Y		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_MAG_Z: //CAL_MAG_Z / Magnetometer Characteristic Z Parameter String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B7B7Z//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B7B7Z		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_TEMPC: //CAL_TEMPC/ Calibration Temperature C String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:B8B8//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	B8B8		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_PRESSURE: //CAL_PRESSURE/ Calibration Pressure String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:BEBE//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	BEBE		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_IRRADIANCE: //CAL_IRRADIANCE / Calibration irradiance String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:BBBB//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	BBBB		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_CAP_SENSE: //CAL_CAP_SENSE / Calibration Cap Sense Event Frequency String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:BCBC//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	BCBC		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_SWPT_FREQ: //CAL_SWPT_FREQ / Calibration Swept Frequency String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:BDBDI//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	BDBDI		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_SWPT_LEVL: //CAL_SWPT_LEVL / Calibration Swept Frequency Level String
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:BDBDL//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                        SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	BDBDL		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetSlope( (Cal_Characteristic)x ),
                                      SkyPack_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                          } // EndSwitch(x)
                          // Now Print String.
                          Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
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
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TCT_SYNTAX</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          strcpy( (char *)tempBffr2, "TCT SYNTAX ERROR: Not correct format.\r\n");
                        } // Endif (tempBffr[3]!=':')
                        else
                        {
                          // 2. Verify if remaining string is digits
                          if (Size <= 4)
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_TCT_BADPARAM</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            strcpy( (char *)tempBffr2, "TCT SYNTAX ERROR: Bad Parameter.\r\n");
                          } // EndIf (Size > 4)
                          else
                          {
                            // 3. Grab remaining string and Save it.
                            tempPstr = &tempBffr[4];
                            strcpy(tempstr, tempPstr);
                            // NOW...Save it.
                            Status = SkyPack_CAL_Set_TimeString( (uint8_t *)tempPstr );
                            if (Status != HAL_OK)
                            {
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>CMD_TCT_ERR</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              return Status;
                            }
                            else
                            {
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>ST_TCT_ACK</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                            }
                            sprintf( (char *)tempBffr2, "\r\n     COMPLETE.\r\n" );
                          } // EndElse (Size > 4)
                        } // EndElse (tempBffr[3]!=':')
                        break;
                        //------------------ TCI Command: Calibration Initialize Cal Table(Reset)
                      case 'I':
                        Status = SkyPack_CAL_InitializeFrmFlash();
                        if (Status != HAL_OK)
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TCI_ERR</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          return Status;
                        }
                        else
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>ST_TCI_ACK</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                        }
                        sprintf( (char *)tempBffr2, "\r\n     COMPLETE.\r\n" );
                        break;
                      } //EndSwitch
                    } //EndElse (Size == 2)
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
