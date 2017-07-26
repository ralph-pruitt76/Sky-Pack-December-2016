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
//#include "Calibration.h"

static bool Bypass = false;

/* Parser function */

/**
  * @brief  This routine parses the passed string and performs the passed operation
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef SkyBrd_ParseString(char *tempBffr)
{
    #define RECEIVE_SZ      30
    uint16_t DriverStatus;
    int8_t tempBffr2[120];
//    int8_t tempBffr3[10];
//    int8_t* BufferPntr;
    HAL_StatusTypeDef Status;
//    HAL_StatusTypeDef Save_Status;
    uint8_t Size;
//    int Address;
//    int num_bytes;
//    int num_bytes_received;
//    uint8_t i2cData[80];
    int x;
//    int Error, y;
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
            case 'D':
              // Read Humidity. 
              strcpy( (char *)tempBffr2, "TBD: Read Humidity NOT IMPLEMENTED\r\n");
/*              if (Size == 1)
              {
//------------------ D Command: Read Humidity Values      
                // Read Humidity Sensor sensor and return Humidity results....
                Status = RoadBrd_Humidity_ReadHumidity( &HMeasure );
                if (Status == HAL_OK)
                  Status = RoadBrd_Humidity_ReadHumidity_Scaled( &HMeasureScaled );
                if (Status == HAL_OK)
                {
                  // Send string to UART..
                  strcpy( (char *)tempBffr2, "Humidity SENSOR...\r\n");
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  // NOW, Build Data String..
                  sprintf( (char *)tempBffr2, "     Humidity DATA: " );
                  strcat( (char *)tempBffr2, (char *)HMeasure.HRaw );
                  strcat( (char *)tempBffr2, "\r\n" );
                }
                else
                  break;
#ifdef NUCLEO
                Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                if (Status != HAL_OK)
                  return Status;
                // NOW, Build Data String..
                sprintf( (char *)tempBffr2, "     Humidity DATA(Decimal): %d\r\n", HMeasure.HRawC );
#ifdef NUCLEO
                Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                if (Status != HAL_OK)
                  return Status;
                // Now calculate Celcius and Farenheit Temp.
                sprintf( (char *)tempBffr2, "     Humidity: %s/%s\r\n", (char *)HMeasure.Humidity, (char *)HMeasureScaled.Humidity );
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ DI Command: Initialize Humidity Sensor
                  case 'I':
                    // Initialize Humidity Sensor.
                    Status = RoadBrd_HumidityInit();
                    if (Status == HAL_OK)
                    {
                      strcpy( (char *)tempBffr2, "Humidity Sensor: Initialization Complete.\r\n");;
                    }
                    break;
//------------------ D0 Command...Read Humidity Values.....
                  case '0':
                    // Read Humidity Sensor sensor and return Humidity results....
                    Status = RoadBrd_Humidity_ReadHumidity( &HMeasure );
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Humidity SENSOR...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, "     Humidity DATA: " );
                      strcat( (char *)tempBffr2, (char *)HMeasure.HRaw );
                      strcat( (char *)tempBffr2, "\r\n" );
                    }
                    else
                      break;
#ifdef NUCLEO
                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                    if (Status != HAL_OK)
                      return Status;
                    // NOW, Build Data String..
                    sprintf( (char *)tempBffr2, "     Humidity DATA(Decimal): %d\r\n", HMeasure.HRawC );
#ifdef NUCLEO
                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                    if (Status != HAL_OK)
                      return Status;
                    // Now calculate Humidity.
                    sprintf( (char *)tempBffr2, "     Humidity: " );
                    strcat( (char *)tempBffr2, (char *)HMeasure.Humidity );
                    strcat( (char *)tempBffr2, "\r\n" );
                    break;
//------------------ D1 Command...Read Temperature Values..... 
                  case '1':
                    // Read Humidity Sensor sensor and return Temperature results....
                    Status = RoadBrd_Humidity_ReadTemperature( &TMeasure );
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Humidity SENSOR...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, "     TEMP DATA: " );
                      strcat( (char *)tempBffr2, (char *)TMeasure.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
                    }
                    else
                      break;
#ifdef NUCLEO
                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                    if (Status != HAL_OK)
                      return Status;
                    // NOW, Build Data String..
                    sprintf( (char *)tempBffr2, "     TEMP DATA(Decimal): %d\r\n", TMeasure.RawC );
#ifdef NUCLEO
                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                    if (Status != HAL_OK)
                      return Status;
                    // Now calculate Celcius and Farenheit Temp.
                    sprintf( (char *)tempBffr2, "     TempC: " );
                    strcat( (char *)tempBffr2, (char *)TMeasure.TempC );
                    strcat( (char *)tempBffr2, "     TempF: " );
                    strcat( (char *)tempBffr2, (char *)TMeasure.TempF );
                    strcat( (char *)tempBffr2, "\r\n" );
                    break;
                } //EndSwitch
              } //EndElse (Size == 1) */
              break; 
//**************************************************************************************************
            case 'E':
              // Read Temp and Pressure. 
              strcpy( (char *)tempBffr2, "TBD: Read Temp and Pressure NOT IMPLEMENTED\r\n");
              break;
//**************************************************************************************************
            case 'F':
              // NO ACTION. 
              strcpy( (char *)tempBffr2, "NO Action...(0x00).\r\n");
              break;
//**************************************************************************************************
            case 'G':
              strcpy( (char *)tempBffr2, "TBD: Read Temperature NOT IMPLEMENTED\r\n");
              // Read Temperature sensor and return results....Temperature Sensor U10(PCT2075GVJ).  Addr: 0x94
/*              Status = RoadBrd_ReadTemp( &TMeasure );
              if (Status == HAL_OK)
                Status = RoadBrd_ReadTemp_Scaled( &TMeasureScaled );
              if (Status == HAL_OK)
              {
                // Send string to UART..
                strcpy( (char *)tempBffr2, "TEMP SENSOR...\r\n");
#ifdef NUCLEO
                Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                if (Status != HAL_OK)
                  return Status;
                // NOW, Build Data String..
                sprintf( (char *)tempBffr2, "     TEMP DATA: " );
                strcat( (char *)tempBffr2, (char *)TMeasure.Raw );
                strcat( (char *)tempBffr2, "\r\n" );
              }
              else
                break;
#ifdef NUCLEO
              Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
              Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
              if (Status != HAL_OK)
                return Status;
              // NOW, Build Data String..
              sprintf( (char *)tempBffr2, "     TEMP DATA(Decimal): %d\r\n", TMeasure.RawC );
#ifdef NUCLEO
              Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
              Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
              if (Status != HAL_OK)
                return Status;
              // Now calculate Celcius and Farenheit Temp.
              sprintf( (char *)tempBffr2, "     TempC: %s/%s     TempF: %s/%s\r\n", 
                      (char *)TMeasure.TempC, 
                      (char *)TMeasureScaled.TempC,
                      (char *)TMeasure.TempF, 
                      (char *)TMeasureScaled.TempF); */
              break;
//**************************************************************************************************
            case 'H':
              // RGB Color Light Sensor U15(ISL91250).  Addr: 0x88
              strcpy( (char *)tempBffr2, "TBD: Read Illuminance NOT IMPLEMENTED\r\n");
/*              if (Size == 1)
              {
//------------------ H Command...Read RGB Values and Return as 3 (2 Byte Fields)....REDmsb,REDlsb,GREENmsb,GREENlsb,BLUEmsb,BLUElsb.....     
                // 1. Time to send Command and collect status.
                Status = RoadBrd_RGBReadValues( &RGBValues );
                if (Status == HAL_OK)
                {
                  // Send string to UART..
                  strcpy( (char *)tempBffr2, "RGB Color Light Sensor...\r\n");
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  // NOW, Build Data String..
                  sprintf( (char *)tempBffr2, " RGB(REDmsb,REDlsb,GREENmsb,GREENlsb,BLUEmsb,BLUElsb)\r\n" );
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  strcpy( (char *)tempBffr2, "       DATA: ");
                  strcat( (char *)tempBffr2, (char *)RGBValues.Raw );
                  strcat( (char *)tempBffr2, "\r\n" );
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  // Now DisplayEach Value Calculated.
                  strcpy( (char *)tempBffr2, "    Red: ");
                  strcat( (char *)tempBffr2, (char *)RGBValues.Red );
                  strcat( (char *)tempBffr2, "    Green: ");
                  strcat( (char *)tempBffr2, (char *)RGBValues.Green );
                  strcat( (char *)tempBffr2, "    Blue: ");
                  strcat( (char *)tempBffr2, (char *)RGBValues.Blue );
                  strcat( (char *)tempBffr2, "\r\n" );
                }
                else
                  break;
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ HI Command...Initialize RGB Color Light Sensor.....     
                  case 'I':
                    if (Size == 2)
                    {
                      // This is the default init. Assume Default Parms and write them.
                      Status = RoadBrd_RGBInit();

                      if (Status == HAL_OK)
                      {
                        strcpy( (char *)tempBffr2, "RGB Color Light Sensor: Initialization Complete with DEFAULT Values.\r\n");;
                      }
                      
                    }
                    else
                    {
                      // This is the Parameter init. Will have to verify all parameters first.
                      if ( Size != 29 )
                      {
                        strcpy( (char *)tempBffr2, "RGB Color Light Sensor: SYNTAX Error. Parameters are not correct.\r\n");;
                      }
                      else
                      {
                        // Step 1. Validate format.
                        if( (tempBffr[2]!=':') ||
                            (tempBffr[5]!='.') || 
                            (tempBffr[8]!='.') || 
                            (tempBffr[11]!='.') || 
                            (tempBffr[14]!='.') || 
                            (tempBffr[17]!='.') || 
                            (tempBffr[20]!='.') || 
                            (tempBffr[23]!='.') || 
                            (tempBffr[26]!='.') )
                        {
                          strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: Not correct format. Punctuation!\r\n");
                        }
                        else
                        {
                          // Syntax correct. Time to grab parameters.
                          Error = 0;
                          for (x=0; x<9; x++)
                          {
                            tempBffr3[0] = tempBffr[3+x*3];
                            tempBffr3[1] = tempBffr[4+x*3];
                            tempBffr3[2] = 0x00;
                            if (isHexNum( (char *)tempBffr3 ) == 0)
                            {
                              strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR:Parameters not HEX Value.\r\n");
                              Error = 1;
                              break;
                            }
                            else
                            {
                              i2cData[x] =  hatoi( (char *)tempBffr3 );
                            } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)
                          } //EndFor (x=0; x<num_bytes; x++)
                          // Format and parameters now in i2cData array. Pull them out and validate them.
                          // OP_MODE Verify.
                          if(i2cData[0]>7)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: OP_MODE.\r\n");
                            break;
                          }
                          else
                            op_mode = (i2cData[0] & 0x07) * 1;
                          // DS_RANGE Verify.
                          if(i2cData[1]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: DS_RANGE.\r\n");
                            break;
                          }
                          else
                            ds_range = (i2cData[1] & 0x01) * 8;
                          // ADC_RSL Verify.
                          if(i2cData[2]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: ADC_RSL.\r\n");
                            break;
                          }
                          else
                            adc_rsl = (i2cData[2] & 0x01) * 16;
                          // SYNC Verify.
                          if(i2cData[3]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: SYNC.\r\n");
                            break;
                          }
                          else
                            sync = (i2cData[3] & 0x01) * 32;
                          // CMP_ADJST Verify.
                          if(i2cData[4]>63)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: CMP_ADJST.\r\n");
                            break;
                          }
                          else
                            cmp_adjst = (i2cData[4] & 0x3f) * 1;
                          // CMP_OFFST Verify.
                          if(i2cData[5]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: CMP_OFFST.\r\n");
                            break;
                          }
                          else
                            cmp_offst = (i2cData[5] & 0x01) * 128;
                          // INT_ASSGN Verify.
                          if(i2cData[6]>3)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: INT_ASSGN.\r\n");
                            break;
                          }
                          else
                            int_assgn = (i2cData[6] & 0x03) * 1;
                          // INT_PERSIST Verify.
                          if(i2cData[7]>3)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: INT_PERSIST.\r\n");
                            break;
                          }
                          else
                            int_persist = (i2cData[7] & 0x03) * 4;
                          // CNVRSN_INT Verify.
                          if(i2cData[8]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: CNVRSN_INT.\r\n");
                            break;
                          }
                          else
                            cnvrsn_int = (i2cData[8] & 0x01) * 16;
                          // OK, all parameters have been verified. Time to build final params.
                          RGBMeasure.config1 = op_mode + ds_range + adc_rsl + sync;
                          RGBMeasure.config2 = cmp_adjst + cmp_offst;
                          RGBMeasure.config3 = int_assgn + int_persist + cnvrsn_int;
                          // Load Config Register with Config Settings
                          Status = RoadBrd_RGBFullInit( &RGBMeasure );

                          if (Status == HAL_OK)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor: Initialization Complete with USER Values.\r\n");;
                          }
                        } //ElseIf Validate format.
                      } //ElseIf ( Size != 29 )
                    } //ElseIf (Size == 2)
                    break;
//------------------ H0 Command...Read RGB Values and Return as 3 (2 Byte Fields)....REDmsb,REDlsb,GREENmsb,GREENlsb,BLUEmsb,BLUElsb.....     
                  case '0':
                    // 1. Time to send Command and collect status.
                    Status = RoadBrd_RGBReadValues( &RGBValues );
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "RGB Color Light Sensor...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, " RGB(REDmsb,REDlsb,GREENmsb,GREENlsb,BLUEmsb,BLUElsb)\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      strcpy( (char *)tempBffr2, "       DATA: ");
                      strcat( (char *)tempBffr2, (char *)RGBValues.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // Now DisplayEach Value Calculated.
                      strcpy( (char *)tempBffr2, "    Red: ");
                      strcat( (char *)tempBffr2, (char *)RGBValues.Red );
                      strcat( (char *)tempBffr2, "    Green: ");
                      strcat( (char *)tempBffr2, (char *)RGBValues.Green );
                      strcat( (char *)tempBffr2, "    Blue: ");
                      strcat( (char *)tempBffr2, (char *)RGBValues.Blue );
                      strcat( (char *)tempBffr2, "\r\n" );
                    }
                    else
                      break;
                    break;
//------------------ H1 Command...Read Status.....     
                  case '1':
                    // 1. Time to send Command and collect status.  RGBSMeasure
                    Status = RoadBrd_RGBReadStatus( &RGBSMeasure );

                    if (Status == HAL_OK)
                    {
                      // Build Status
                      strcpy( (char *)tempBffr2, "RGB Color Light Sensor Status: ");
                      sprintf( (char *)tempBffr3, "%02x / ", RGBSMeasure.status);
                      strcat( (char *)tempBffr2, (char *)tempBffr3 );
                      strcat( (char *)tempBffr2, (char *)RGBSMeasure.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
                    }
                    else
                      break;
                    break;
//------------------ H2 Command...Reset Hardware......     
                  case '2':
                    Status = RoadBrd_RGBReset();
                    if (Status == HAL_OK)
                    {
                      // Build Status
                      strcpy( (char *)tempBffr2, "RGB Color Light Sensor: RESET CMD Sent Succesful.\r\n" );
                    }
                    else
                      break;
                    break;
//------------------ H3 Command...Read ID.....     
                  case '3':
                    // 1. Time to send Command and collect status.  IDMeasure
                    Status = RoadBrd_RGBReadID( &IDMeasure );
                    if (Status == HAL_OK)
                    {
                      // Build Status
                      strcpy( (char *)tempBffr2, "RGB Color Light Sensor ID Code: ");
                      sprintf( (char *)tempBffr3, "%02x / ", IDMeasure.id);
                      strcat( (char *)tempBffr2, (char *)tempBffr3 );
                      strcat( (char *)tempBffr2, (char *)IDMeasure.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
                   }
                    else
                      break;
                    break;
                  default:
                    strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                    break;
                } //EndSwitch
              } //EndElse (Size == 1) */
              break;    
//**************************************************************************************************
            case 'P':
              // POWER SYSTEM. 
              strcpy( (char *)tempBffr2, "TBD: POWER SYSTEM CMDS NOT IMPLEMENTED\r\n");
/*              switch( tempBffr[1] )
              {
//++++++++++++++++++++++++++++++++++++++++++  5V Power Supply Commands.
                case 'U':
                  // Turn on 5V Power Supply.
                  RoadBrd_gpio_On( gTAM_PWR );
                  strcpy( (char *)tempBffr2, "5V Power Plane Powered UP.\r\n");
                  break;
                case 'D':
                  // Turn off 5V Power Supply.
                  RoadBrd_gpio_Off( gTAM_PWR );
                  strcpy( (char *)tempBffr2, "5V Power Plane Powered DOWN.\r\n");
                  break;
                default:
                  strcpy( (char *)tempBffr2, "ERROR: Illegal 5V Power Plane Command.\r\n");
                  break;
              } */
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
/*                    // I2C Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    if (Size < 9)
                    strcpy( (char *)tempBffr2, "TI SYNTAX ERROR: Not correct format.\r\n");
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
                            strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
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
                                Status =  RoadBrd_I2C_Master_Transmit((uint16_t)Address, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
                                
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
                            strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
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
                                  // 6. Time to send Command and collect status.
                                  Status =  RoadBrd_I2C_Master_Transmit((uint16_t)Address, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
                                  // 6a. Wait for Command to complete(100ms).
 
                                  // 7. If Status was good, Time to get response.
                                  if (Status == HAL_OK)
                                  {
                                    Status =  RoadBrd_I2C_Master_Receive((uint16_t)Address, i2cData, (uint16_t)num_bytes_received, I2C_TIMEOUT);
                                  }
                                  else
                                    break;
                                  // 7a. Wait for Command to complete(100ms).
                                  if (Status == HAL_OK)
                                  {
                                    Status = RoadBrd_WaitForState( 20 );
                                  }

                                  // 8. IfGood report, Need to Output Data.
                                  if (Status == HAL_OK)
                                  {
                                    // Send string to UART..
#ifdef NUCLEO
                                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
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
//------------------
                        case 'Q':
                          //I2C Receive Command
                          // Step 1. Validate format.
                          if( (tempBffr[3]!=':') ||
                              (tempBffr[6]!='.')  )
                          {
                            strcpy( (char *)tempBffr2, "TIQ SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
                            // 2. Grab Address and validate a legal number
                            tempBffr3[0] = tempBffr[4];
                            tempBffr3[1] = tempBffr[5];
                            tempBffr3[2] = 0x00;
                            if (isHexNum( (char *)tempBffr3 ) == 0)
                              strcpy( (char *)tempBffr2, "TIQ SYNTAX ERROR: Address not HEX Value.\r\n");
                            else
                            {
                              // Legal Address. Save it as value
                              Address = hatoi( (char *)tempBffr3 );
                              // 3. Now get the number of bytes Received of data from field.
                              tempBffr3[0] = tempBffr[7];
                              tempBffr3[1] = tempBffr[8];
                              tempBffr3[2] = 0x00;
                              if (isHexNum( (char *)tempBffr3 ) == 0)
                                strcpy( (char *)tempBffr2, "TIQ SYNTAX ERROR: Number of Bytes not HEX Value.\r\n");
                              else
                              {
                                  // Legal NUMBER BYTES. Save it as value
                                  num_bytes_received = hatoi( (char *)tempBffr3 );
                                  // 4. Test num_bytes. If Zero, We are done
                                  sprintf( (char *)tempBffr2, "TIR: GOOD CMD: %x.\r\n", Address);
                                  // 7. If Status was good, Time to get response.
                                  Status =  RoadBrd_I2C_Master_Receive((uint16_t)Address, i2cData, (uint16_t)num_bytes_received, I2C_TIMEOUT);
                                  // 7a. Wait for Command to complete(100ms).
                                  if (Status == HAL_OK)
                                  {
                                    Status = RoadBrd_WaitForState( 20 );
                                  }
                                  else
                                    break;
                                  // 8. IfGood report, Need to Output Data.
                                  if (Status == HAL_OK)
                                  {
                                    // Send string to UART..
#ifdef NUCLEO
                                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
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
                              } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...NUMBER BYTES
                              
                            } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...Address
                            
                          } //EndElse ( (tempBffr[2]!=':') || (tempBffr[5]!='.') )
                      
                          break;
                        default:
                          strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                          break;
                      } //EndSwitch ( tempBffr[2] )
                    } //EndElse (Size < 9) */
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
                          strcpy( (char *)tempBffr2, "TCS SYNTAX ERROR: Not correct format.\r\n");
                        } // Endif (tempBffr[3]!=':')
                        else
                        {
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
                          strcpy( (char *)tempBffr2, "TCT SYNTAX ERROR: Not correct format.\r\n");
                        } // Endif (tempBffr[3]!=':')
                        else
                        {
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
                    sprintf( (char *)tempBffr2, "Driver Status: %04x\r\n", DriverStatus );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  RESET Micro.
                  case 'R':
                    // Read Driver Status
                    strcpy( (char *)tempBffr2, "TR CMD: Reset Micro NOT IMPLEMENTED\r\n");
/*                    HAL_NVIC_SystemReset();
                    sprintf( (char *)tempBffr2, "RESET CALLED BUT NO RESPONSE!!\r\n" );*/
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Key Flash Variable Commands.
                  case 'K':
                    // Key Flash Variable Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    Status = HAL_OK;
                    if (Size < 4)
                      strcpy( (char *)tempBffr2, "TK SYNTAX ERROR: Not correct format.\r\n");
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
                                    strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Bad Parameter.\r\n");
                                  }
                                  else
                                  {
                                    // Time to set new Road Sound Sample Rate.
#ifdef STM32L151CBT6
                                    SkyBrd_Set_SnsrTickCnt( new_value );
#endif
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "COMPLETE" );
                                  } // EndElse ((new_value > 9999) || (new_value < 0))
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
                          //Key Flash Variable Set Command.
                          switch( tempBffr[3] )
                          {
//------------------
                            case 'S':
                              //Key Flash Variable Read Sensor Sample Rate Command.
#ifdef STM32L151CBT6
                              sprintf( (char *)tempBffr2, "Sensor Sample Rate: %3.1f Seconds.\r\n\r\n> ", ((float)SkyPack_GetSampleTime()/10));
#else
                              sprintf( (char *)tempBffr2, "Sensor Sample Rate: (NOT SUPPORTED) Seconds.\r\n\r\n> " );
#endif
                              break;
                            default:
                              strcpy( (char *)tempBffr2, "TKS ERROR: Not a legal command.\r\n");
                              break;
                          } // EndSwitch ( tempBffr[3] )
                          break;
                        default:
                          strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                          break;
                      } //EndSwitch ( tempBffr[2] )
                    } //EndElse (Size < 3)
                    break;
                  default:
                    // ERROR if we get here.. 
                    strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                    break;
                 
                } //EndSwitch ( tempBffr[1] )
              } //EndElse (Size <= 1)
              break;
            default:
              // ERROR if we get here.. 
              strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
              break;
            } // EndSwitch
            

            // Test last I2C Status to determine next msg.
/*            switch( Status )
            {
              case HAL_OK:
                break;
              case HAL_ERROR:
                // Determine what kind of error.
                Err_code = RoadBrd_I2C_GetError();
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
                MX_I2C1_Reset();
                break;
              case HAL_BUSY:
                strcpy( (char *)tempBffr2, "I2C BUSY: I2C reported BUSY error.\r\n");
                // Re-Initialize I2C....It has been corrupted.
                MX_I2C1_Reset();
                break;
              case HAL_TIMEOUT:
                strcpy( (char *)tempBffr2, "I2C TIMEOUT: I2C reported TIMEOUT.\r\n");
                // Re-Initialize I2C....It has been corrupted.
                MX_I2C1_Reset();
                break;
              default:  
                strcpy( (char *)tempBffr2, "I2C ERROR: I2C reported unnown error.\r\n");
                // Re-Initialize I2C....It has been corrupted.
                MX_I2C1_Reset();
                break;
            } */
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
