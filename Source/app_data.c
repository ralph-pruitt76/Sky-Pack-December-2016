/* Application data manager implementation
 * Takes care of scheduling sample times and formatting the sensor data
 * to be sent as BLE characteristics to the app. */

#include "app_data.h"
#include "lsm6ds3.h"
#include "lis3mdl.h"
#include "lps25hb.h"
#include "pct2075.h"
#include "opt3001.h"
#include "misc_sense.h"
#include "bgm111.h"
#include <stdio.h>
#include <string.h>
#include "sys_ctrl.h"
#include "ErrCodes.h"
#include "miscRoutines.h"

/* Characteristic handles */
/*
#define gattdb_accelerometer                    8
#define gattdb_magnetometer                    12
#define gattdb_temperature                     16
#define gattdb_gyro                            20
#define gattdb_irradiance                      24
#define gattdb_wiper_freq                      28
#define gattdb_swept_freq_idx                  32
#define gattdb_pressure                        36
#define gattdb_xgatt_rev                       40
#define gattdb_AnlErrCnt                       45
#define gattdb_AnlErrCd                        49
#define gattdb_AnlDevCd                        53
#define gattdb_AnlTickCnt                      57
#define gattdb_AnlHrtBt                        61
#define gattdb_AnlHrtBt2                       65
*/
#define gattdb_xgatt_spp_data                   13

/* App data measurment structure */

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

// Driver list Structure
struct
{
  bool  Irradiance;
  bool  Pressure;
  bool  IMUSense;
  bool  I2CState;
} static driver_list;

// Analytics Structure
struct
{
  bool  HrtBeat_Flg;
  uint16_t HrtBeat_Cnt;
} static analytics;

/* App data measurment structure */

typedef struct
{
  volatile bool         reading_scheduled;
  bool          Legacy_OneTime;
  struct {
    union u3DVector     accel;
    union u3DVector     gyro;
    union u3DVector     mag;
  } imu;
  int16_t               temperature;
  int16_t               pressure;
  int32_t               irradiance;
  struct {
    uint16_t            event_freq;
    uint16_t            swept_idx;
    uint16_t            swept_level;
  } cap;
} dataTmplate;

static dataTmplate TmpData;
static dataTmplate data;

static uint16_t connection_cnt = 0;
static uint16_t HeartBeat_Cnt = 0;
static int HeartCnt = 0;
 
/* Initialize the IMU sensors */

HAL_StatusTypeDef InitIMUSensors(void)
{
  IMU_6AXES_InitTypeDef LSM6DS3_InitStructure;
  MAGNETO_InitTypeDef LIS3MDL_InitStructure;
  
  /* Set up the accelerometer/gyro init structure
   * The driver is fancy and expects values, not definitions */
  LSM6DS3_InitStructure.X_OutputDataRate = 10.0f;
  LSM6DS3_InitStructure.X_FullScale = 4.0f;
  LSM6DS3_InitStructure.X_X_Axis = LSM6DS3_XL_XEN_ENABLE;
  LSM6DS3_InitStructure.X_Y_Axis = LSM6DS3_XL_YEN_ENABLE;
  LSM6DS3_InitStructure.X_Z_Axis = LSM6DS3_XL_ZEN_ENABLE;
  LSM6DS3_InitStructure.G_OutputDataRate = 10.0f;
  LSM6DS3_InitStructure.G_FullScale = 500.0f;
  LSM6DS3_InitStructure.G_X_Axis = LSM6DS3_G_XEN_ENABLE;
  LSM6DS3_InitStructure.G_Y_Axis = LSM6DS3_G_YEN_ENABLE;
  LSM6DS3_InitStructure.G_Z_Axis = LSM6DS3_G_ZEN_ENABLE;
  
  /* Initialize the accelerometer/gyro */
  if(LSM6DS3_Init(&LSM6DS3_InitStructure) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set up the magnetometer init structure */
  LIS3MDL_InitStructure.M_OutputDataRate = LIS3MDL_M_DO_10;
  LIS3MDL_InitStructure.M_OperatingMode = LIS3MDL_M_MD_CONTINUOUS;
  LIS3MDL_InitStructure.M_FullScale = LIS3MDL_M_FS_4;
  LIS3MDL_InitStructure.M_XYOperativeMode = LIS3MDL_M_OM_UHP;
  
  /* Initialize the magnetometer */
  if(LIS3MDL_Init(&LIS3MDL_InitStructure) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/* Read the IMU sensor data
 * - Accelerometer data is in 1/1000 g
 * - Gyro data is in 1/10 dps
 * - Magnetometer data is in mGauss
 * All scaling done according to typical sensitivity values in datasheet */

HAL_StatusTypeDef ReadIMUSensors(void)
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
      TmpData.imu.accel.indexed[i] = (int16_t)
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
      TmpData.imu.gyro.indexed[i] = (int16_t)
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
      TmpData.imu.mag.indexed[i] = (int16_t)
                        (((int32_t)(vector.indexed[i]) * 4000) / (6842 * 4));
    }
  }
  else
    return HAL_ERROR;
  
  return HAL_OK;
}

/* Initialize the pressure/temperature sensor */

HAL_StatusTypeDef InitPressureTempSensor(void)
{
  PRESSURE_InitTypeDef LPS25HB_InitStructure;
  PCT2075_InitTypeDef PCT2075_InitStructure;
  
  /* Set up the pressure/temperature sensor init structure */
  LPS25HB_InitStructure.OutputDataRate = LPS25HB_ODR_7Hz;
  LPS25HB_InitStructure.PressureResolution = LPS25HB_P_RES_AVG_32;
  LPS25HB_InitStructure.TemperatureResolution = LPS25HB_T_RES_AVG_32;
  LPS25HB_InitStructure.DiffEnable = LPS25HB_DIFF_DISABLE;
  LPS25HB_InitStructure.BlockDataUpdate = LPS25HB_BDU_CONT;
  LPS25HB_InitStructure.SPIMode = LPS25HB_SPI_SIM_4W;
  
  /* Initialize the pressure/temperature sensor */
  if(LPS25HB_Init(&LPS25HB_InitStructure) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Set up the separate temperature sensor init structure */
  PCT2075_InitStructure.Device_Mode = PCT2075_DEVMODE_NORMAL;
  PCT2075_InitStructure.OS_Queue = PCT2075_OSFQUEUE_1;
  PCT2075_InitStructure.OS_Polarity = PCT2075_OSPOL_LOW;
  PCT2075_InitStructure.OS_Mode = PCT2075_OSMODE_COMP;
  PCT2075_InitStructure.OS_Trigger = 0.0f;
  PCT2075_InitStructure.OS_Hyst = 0.0f;
  PCT2075_InitStructure.Idle_Time = 2;
  
  /* Initialize the separate temperature sensor */
  if(PCT2075_Init(&PCT2075_InitStructure) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/* Read the pressure and temperature data
 * - Pressure is in 1/10 mbar
 * - Temperature is in 1/10 degree celcius */

HAL_StatusTypeDef ReadPressureTempSensors(void)
{
  float value;
  
  /* Read the temperature */
  if (LPS25HB_GetTemperature(&value) == HAL_OK)
  {
    /* Scale the value so it is in 1/10 C */
    TmpData.temperature = (int16_t)(value * 10);
  }
  else
  {
    /* If that failed, try the separate temperature sensor */
    TmpData.temperature = (int16_t)(PCT2075_GetTemp() * 10);
  }
  
  /* Read the pressure */
  if (LPS25HB_GetPressure(&value) == HAL_OK)
  {
    /* Scale the value so it is in 1/10 mbar */
    TmpData.pressure = (int16_t)(value * 10);
  }
  else
    return HAL_ERROR;
  
  return HAL_OK;
}

/* Initialize the irradiance sensor */

HAL_StatusTypeDef InitIrradianceSensor(void)
{
  OPT3001_InitTypeDef OPT3001_InitStructure;
  
  /* Set up the irradiance sensor init structure */
  OPT3001_InitStructure.Range = OPT3001_CONFIG_RN_AUTO;
  OPT3001_InitStructure.Conversion_Time = OPT3001_CONFIG_CT_100MS;
  OPT3001_InitStructure.Mode = OPT3001_CONFIG_M_CONTINUOUS;
  OPT3001_InitStructure.Int_Latch = OPT3001_CONFIG_L_HYST;
  OPT3001_InitStructure.Int_Pol = OPT3001_CONFIG_POL_LOW;
  OPT3001_InitStructure.Fault_Count = OPT3001_CONFIG_FC_1;
  OPT3001_InitStructure.Low_Limit = 0;
  OPT3001_InitStructure.High_Limit = 0xFFFFFFFF;

  /* Initialize the irradiance sensor */
  return (OPT3001_Init(&OPT3001_InitStructure));
}

/* Initialize sample timer */

void InitSampleTimer(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable clock for the sample timer */
  RCC_APB1PeriphClockCmd(SAMPLE_TIM_RCC, ENABLE);

  /* OLD: Configure to generate an interrupt every 200ms */
  /* BRONZE: Configure to generate an interrupt every 1000ms */
  /* BRONZE.2: Configure to generate an interrupt every 10000ms */
  TIM_TimeBaseInitStructure.TIM_Prescaler = 32 * 1000;
  TIM_TimeBaseInitStructure.TIM_Period = 10000;
  TIM_TimeBaseInit(SAMPLE_TIM, &TIM_TimeBaseInitStructure);

  /* Enable and configure sample timer update interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = SAMPLE_TIM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ITConfig(SAMPLE_TIM, TIM_IT_Update, ENABLE);

  /* Enable the sample timer */
  TIM_Cmd(SAMPLE_TIM, ENABLE);
}

/* Read the miscellaneous sensors */

void ReadMiscSensors(void)
{
  /* Read the cap sense event frequency */
  TmpData.cap.event_freq = GetCapSenseEventFreq();
  TmpData.cap.swept_idx = GetSweptFreqHighIdx();
  TmpData.cap.swept_level = GetSweptFreqHighLevel();
}

/* Initialize all sensors */

void InitSensors(void)
{
  HAL_StatusTypeDef Status;

  data.Legacy_OneTime = true;                   // Clear Legacy One time flag so that we can set key characteristics...once.
  
  // Preset some data to force a sample.
  data.imu.accel.named.x = 0xffff;
  data.imu.gyro.named.x = 0xffff;
  data.imu.mag.named.x = 0xffff;
  data.temperature = 0xffff;
  data.pressure = 0xffff;
  data.irradiance = 0xffff;
  data.cap.event_freq = 0xffff;
  data.cap.swept_idx = 0xffff;

  /* Initialize the I2C peripheral */
  I2C_LowLevel_Init();
  
  /* Initialize I2C connected sensors for the application */
  Status = InitIMUSensors();
  if (Status == HAL_OK)
  {
    Set_DriverStates( IMU_STATE_TASK, DRIVER_ON );
  }
  else
  {
    SkPck_ErrCdLogErrCd( ERROR_IMU_INIT, MODULE_AppData );
    Set_DriverStates( IMU_STATE_TASK, DRIVER_OFF );
  }
  Status = InitPressureTempSensor();
  if (Status == HAL_OK)
  {
    Set_DriverStates( PRESSURE_MNTR_TASK, DRIVER_ON );
  }
  else
  {
    SkPck_ErrCdLogErrCd( ERROR_PRESSURE_INIT, MODULE_AppData );
    Set_DriverStates( PRESSURE_MNTR_TASK, DRIVER_OFF );
  }
  Status = InitIrradianceSensor();
  if (Status == HAL_OK)
  {
    Set_DriverStates( IRRADIANCE_MNTR_TASK, DRIVER_ON );
  }
  else
  {
    SkPck_ErrCdLogErrCd( ERROR_ILL_INIT, MODULE_AppData );
    Set_DriverStates( IRRADIANCE_MNTR_TASK, DRIVER_OFF );
  }
  
  /* Initialize misc sensors we run from the micro */
  // Actually. This enables the Cap Sense Code.
#ifndef NO_CAP
  MiscSensors_Init();
#endif
  
  /* Initialize the sample timer */
  InitSampleTimer();
}

/* Sample timer interrupt handler */

void SAMPLE_TIM_IRQHandler(void)
{
  /* Clear the interrupt flag */
  TIM_ClearITPendingBit(SAMPLE_TIM, TIM_IT_Update);
  /* Schedule a sensor reading */
  data.reading_scheduled = true;
  // This tick occurs every 200msec.
  // Turn off the Blue LED NOW.
}

/* Process sensor state machine */

void ProcessSensorState(void)
{
  static int TimeTagCnt = 0;
  HAL_StatusTypeDef Status;
  char characteristic[40];
//  uint8_t tempStr[13];
  uint8_t tempBffr2[40];
  uint8_t tempbffr[30];
  static int nullCnt = 0;
 
  /* Is a reading scheduled? */
  if (data.reading_scheduled)
  {
    // Build Time Tag Every 25th Call to report back to App.
    TimeTagCnt++;
    HeartCnt++;
    if (TimeTagCnt >= 3)
    {
      // Clear Count.
      TimeTagCnt = 0;
      sprintf( (char *)tempBffr2, " \r\n\r\n");
      SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
      sprintf( (char *)tempBffr2, "<TICK>SP/%08x/%04x/%04x></TICK>", HeartCnt, HeartBeat_Cnt, connection_cnt);
      SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      SendApp_String( (uint8_t *)tempBffr2 );
    }
    // Test Connection. Have we timed out??
    Test_Connection();
    
    /* Clear the scheduling flag */
    data.reading_scheduled = false;
    
    /* Read the IMU sensor data */
    if ( Get_DriverStates( IMU_STATE_TASK ))
    {
      Status = ReadIMUSensors();
      if (Status == HAL_OK)
      {
        Set_DriverStates( IMU_STATE_TASK, DRIVER_ON );
      }
      else
      {
        SkPck_ErrCdLogErrCd( ERROR_IMU_ERR, MODULE_AppData );
        Set_DriverStates( IMU_STATE_TASK, DRIVER_OFF );
      }
    }
    /* Read the pressure and temperature */
    if ( Get_DriverStates( PRESSURE_MNTR_TASK ))
    {
      Status = ReadPressureTempSensors();
      if (Status == HAL_OK)
      {
        Set_DriverStates( PRESSURE_MNTR_TASK, DRIVER_ON );
      }
      else
      {
        SkPck_ErrCdLogErrCd( ERROR_PRESSURE_ERR, MODULE_AppData );
        Set_DriverStates( PRESSURE_MNTR_TASK, DRIVER_OFF );
      }
    }
    /* Read the irradiance in 1/100 lux */
    if ( Get_DriverStates( IRRADIANCE_MNTR_TASK ))
    {
      TmpData.irradiance = OPT3001_GetData();
    }
    // Build Display String from value.
    //sprintf( (char *)tempBffr2, "<%5dlx>", data.irradiance);
    sprintf( (char *)tempBffr2, "  <%05dlx/%d>  ", data.irradiance, data.pressure);
    SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2);

    /* Read the miscellaneous cap sensors */
    ReadMiscSensors();
    
    // Verify Data quality by checking irradiance.
    //data.pressure = 0;
    if ( TmpData.pressure == 0)
    {
      // We have null data from irradiance...Inc nullCnt.
      nullCnt++;
      sprintf(( char *)tempbffr, "<<<I2C=NULL:Strike %01d>>>\r\n", nullCnt);
      SkyPack_MNTR_UART_Transmit( (uint8_t *)tempbffr );
      if (nullCnt >= NULL_MAX)
      {
        nullCnt = 0;
        // Oops...Detected a fatal error...RESET!!!
        SkyPack_Reset( FATAL_I2CDROP );
      }
    }
    
    SkyPack_gpio_On(BGM_LED);
    /* Create the accelerometer characteristic string */
    if((TmpData.imu.accel.named.x != data.imu.accel.named.x) ||
       (TmpData.imu.accel.named.y != data.imu.accel.named.y) ||
       (TmpData.imu.accel.named.z != data.imu.accel.named.z) )
    {
       // Update Information in data structure
      data.imu.accel.named.x = TmpData.imu.accel.named.x;
      data.imu.accel.named.y = TmpData.imu.accel.named.y;
      data.imu.accel.named.y = TmpData.imu.accel.named.z;
     
      sprintf(characteristic, "<UB6B6>%05d%05d%05d</UB6B6>", data.imu.accel.named.x,
              data.imu.accel.named.y, data.imu.accel.named.z);
      /* Send the accelerometer data to the BLE module */
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//    BGM111_WriteCharacteristic(gattdb_accelerometer,
//                            strlen(characteristic), (uint8_t *)characteristic);
    }
    /* Create the gyro characteristic string */
    if((TmpData.imu.gyro.named.x != data.imu.gyro.named.x) ||
       (TmpData.imu.gyro.named.y != data.imu.gyro.named.y) ||
       (TmpData.imu.gyro.named.z != data.imu.gyro.named.z) )
    {
       // Update Information in data structure
      data.imu.gyro.named.x = TmpData.imu.gyro.named.x;
      data.imu.gyro.named.y = TmpData.imu.gyro.named.y;
      data.imu.gyro.named.z = TmpData.imu.gyro.named.z;
     
      sprintf(characteristic, "<UB9B9>%05d%05d%05d</UB9B9>", data.imu.gyro.named.x,
              data.imu.gyro.named.y, data.imu.gyro.named.z);
      /* Send the gyro data to the BLE module */
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//    BGM111_WriteCharacteristic(gattdb_gyro,
//                            strlen(characteristic), (uint8_t *)characteristic);
    }
    /* Create the magnetometer characteristic string */
    if((TmpData.imu.mag.named.x != data.imu.mag.named.x) ||
       (TmpData.imu.mag.named.y != data.imu.mag.named.y) ||
       (TmpData.imu.mag.named.z != data.imu.mag.named.z) )
    {
       // Update Information in data structure
      data.imu.mag.named.x = TmpData.imu.mag.named.x;
      data.imu.mag.named.y = TmpData.imu.mag.named.y;
      data.imu.mag.named.z = TmpData.imu.mag.named.z;
     
      sprintf(characteristic, "<UB7B7>%05d%05d%05d</UB7B7>", data.imu.mag.named.x,
              data.imu.mag.named.y, data.imu.mag.named.z);
      /* Send the magnetometer data to the BLE module */
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//    BGM111_WriteCharacteristic(gattdb_magnetometer,
//                            strlen(characteristic), (uint8_t *)characteristic);
    }
    /* Create the temperature characteristic string */
    if(TmpData.temperature != data.temperature)
    {
       // Update Information in data structure
      data.temperature = TmpData.temperature;

      sprintf(characteristic, "<UB8B8>%05d</UB8B8>", data.temperature);
      /* Send the temperature to the BLE module */
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//    BGM111_WriteCharacteristic(gattdb_temperature,
//                            strlen(characteristic), (uint8_t *)characteristic);
    }
    /* Create the pressure characteristic string */
    if(TmpData.pressure != data.pressure)
    {
       // Update Information in data structure
      data.pressure = TmpData.pressure;

      sprintf(characteristic, "<UBEBE>%06d</UBEBE>", data.pressure);
      /* Send the pressure to the BLE module */
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//    BGM111_WriteCharacteristic(gattdb_pressure,
//                            strlen(characteristic), (uint8_t *)characteristic);
    }
    /* Create the irradiance characteristic string */
    if(TmpData.irradiance != data.irradiance)
    {
       // Update Information in data structure
      data.irradiance = TmpData.irradiance;

      sprintf(characteristic, "<UBBBB>%08d</UBBBB>", data.irradiance);
      /* Send the irradiance to the BLE module */
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//    BGM111_WriteCharacteristic(gattdb_irradiance,
//                            strlen(characteristic), (uint8_t *)characteristic);
    }
    /* Create cap sense event frequency characteristic string */
    if(TmpData.cap.event_freq != data.cap.event_freq)
    {
       // Update Information in data structure
      data.cap.event_freq = TmpData.cap.event_freq;

      sprintf(characteristic, "<UBCBC>%05d</UBCBC>", data.cap.event_freq);
      /* Send the cap sense event frequency to the BLE module */
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//    BGM111_WriteCharacteristic(gattdb_wiper_freq,
//                            strlen(characteristic), (uint8_t *)characteristic);
    }
    /* Create swept frequency characteristic string */
    if((TmpData.cap.swept_idx != data.cap.swept_idx) ||
       (TmpData.cap.swept_level != data.cap.swept_level) )
    {
       // Update Information in data structure
      data.cap.swept_idx = TmpData.cap.swept_idx;
      data.cap.swept_level = TmpData.cap.swept_level;
     
      sprintf(characteristic, "<UBDBD>%05d%06d</UBDBD>", data.cap.swept_idx,
                                              data.cap.swept_level);
      /* Send the swept frequency to the BLE module */
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//    BGM111_WriteCharacteristic(gattdb_swept_freq_idx,
//                            strlen(characteristic), (uint8_t *)characteristic);
    // Test Analytics flag and determine if we need to update that characteristic
    }
    if (!(Tst_HeartBeat()))
    {
//      sprintf( (char *)tempStr, "%010dHB", analytics.HrtBeat_Cnt++);
//      BGM111_WriteCharacteristic(gattdb_AnlHrtBt,
//                             strlen((char *)tempStr), (uint8_t *)tempStr);
      Set_HeartBeat();
    }
    // Test Legacy flag to perform one time operations
    if( data.Legacy_OneTime )
    {
      // Clear Flag...We are done.
      data.Legacy_OneTime = false;
      // Update BLE Characteristics
      sprintf( characteristic, "<UBFBF>%s</UBFBF>", (char *)LEGACY_BANNER);
      SkyPack_MNTR_UART_Transmit( (uint8_t *)characteristic );
      BGM111_Transmit((uint32_t)(strlen(characteristic)), (uint8_t *)characteristic);
//      BGM111_WriteCharacteristic(gattdb_xgatt_rev,
//                                 strlen((char *)LEGACY_BANNER), (uint8_t *)LEGACY_BANNER);
    }
    SkyPack_gpio_Off(BGM_LED);          // Turn off BGM LED.
  } // EndIf (data.reading_scheduled)
}

/**
  * @brief  This routine delays for 10 msec and returns
  *         from the BGM111
  * @param  none
  * @retval none
  */
void delay_10ms( void )
{
  uint32_t count = 44000;      // Delay loop for 10msec
  
  while (count != 0)
  {
    count--;
  }
}

/**
  * @brief  This routine delays for 10 msec and returns
  *         from the BGM111
  * @param  none
  * @retval none
  */
void delay_100ms( void )
{
  uint32_t count = 440000;      // Delay loop for 10msec
  
  while (count != 0)
  {
    count--;
  }
}

/**
  * @brief  This routine delays the number of passed parameters and then returns
  *         from the BGM111
  * @param  int value: Value of 100msec ticks to wait.
  * @retval none
  */
void delay_100msec( int value )
{
  while( value > 0)
  {
    delay_100ms();
    value--;
  }
}

/**
  * @brief  This routine blinks the LED.
  * @param  none.
  * @retval none
  */
void Flicker_Led( void )
{
  //SetLED(true);
  SkyPack_gpio_On(BLUE_LED);
  delay_10ms();
  //SetLED(false);
  SkyPack_gpio_Off(BLUE_LED);
  delay_10ms();
}

/**
  * @brief  This routine blinks the LED 3 times and then performs a system reset.
  * @param  int code: Number of time to blink LED.
  * @retval none
  */
void SkyPack_Reset( int code )
{
  int x;
  
  // Alert Monitor of Error
  SkPck_ErrCdLogErrCd( Get_ErrorCode( code ), MODULE_Reset );
  // Alert User by Blinking LED Fast and then wait 1 Second.
  for (x=0; x<code; x++)
  {
    //SetLED(true);
    SkyPack_gpio_On(BLUE_LED);
    delay_100msec(1);
    //SetLED(false);
    SkyPack_gpio_Off(BLUE_LED);
    delay_100msec(1);
  }
  //SetLED(true);
  SkyPack_gpio_On(BLUE_LED);
  delay_100msec(10);
  //SetLED(false
  SkyPack_gpio_Off(BLUE_LED);
  delay_100msec(10);
  // Alternate way to reset....Power Down Power Plane.
#ifndef DISABLE_HARD_REBOOT
  SetUSBPower( USB_POWER_OFF );
  delay_100msec(100);
#endif
  
  // Reset Micro and Start Over...
  NVIC_SystemReset();
}

/**
  * @brief  This function streams the passed string to the App via characteristics..
  * @param  uint8_t *pData
  * @retval None
  */
void SendApp_String( uint8_t *pData )
{
  uint8_t tempBffr3[25];
  uint8_t *tempPtr;
  //int tempval;

  if (BGM111_Ready())
  {
    strncpy( (char *)tempBffr3, (char *)pData, 20);
//    BGM111_WriteCharacteristic(gattdb_AnlHrtBt,
//                              strlen((char *)tempBffr3), (uint8_t *)tempBffr3);
    //tempval = strlen((char *)pData);
    //if (tempval > 20)
    if (strlen((char *)pData) > 20)
      tempPtr = &pData[20];
    else
      tempPtr = &pData[0];
    strncpy( (char *)tempBffr3, (char *)tempPtr, 20);
//    BGM111_WriteCharacteristic(gattdb_AnlHrtBt2,
//                              strlen((char *)tempBffr3), (uint8_t *)tempBffr3);
  }
}

  /**
  * @brief  This function Tests for an active connection.
  * @param  None
  * @retval None
  */
void Test_Connection( void )
{
  uint8_t tempBffr2[40];

  // Test Connection
  if ( BGM111_Connected() )
  {
    // Yes...Clear count
    connection_cnt = 0;
#ifndef DISBALE_HEARTBEAT
    // Test Heart Beat. Has it been cleared?
    if (Tst_HeartBeat())
    {
      // No We need to watch this closely.
      // Test Heart Beat Count and determine if time to reset.
      HeartBeat_Cnt++;
      // Time to Build Status?
      if ( (HeartBeat_Cnt % 5) == 0 )
      {
        sprintf( (char *)tempBffr2, " \r\n<HB:%04x/%08x> ", HeartBeat_Cnt, HeartCnt);
        SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
        SendApp_String( tempBffr2 );
      }
      // Test Heart Beat Count. If expired, reset.
      if (HeartBeat_Cnt > HEARTBEAT_CNT)
      {
        // Has been 30 Seconds....Time to reset Code.
//        HeartBeat_Cnt = 0;
//        Clr_HrtBeat_Cnt();
//        SkyPack_Reset( FATAL_TIMEOUT );
      }
    } // EndIf (Tst_HeartBeat())
    else
    {
      // OK. Clear Count.
      HeartBeat_Cnt = 0;
      // Set Heart Beat Flag for next Sequence.
      //Set_HeartBeat();
    }
    
  } // EndIf ( BGM111_Connected() )
  else
  {
    // No...Increment Count...
    connection_cnt++;
#endif   
    // Wait 90 Seconds before forcing reset.
    if (connection_cnt > CONNECTION_CNT)
    {
//      Clr_HrtBeat_Cnt();
//      SkyPack_Reset( FATAL_TIMEOUT );
    }
  } // EndElse ( BGM111_Connected() )
}

  /**
  * @brief  This function Clears the Heart Beat Count.
  * @param  None
  * @retval bool: Status of Heart Beat Flag
  */
void Clr_HrtBeat_Cnt( void )
{
  analytics.HrtBeat_Cnt = 0;
}

  /**
  * @brief  This function returns the status of the Heart Beat Flag.
  * @param  None
  * @retval bool: Status of Heart Beat Flag
  */
bool Tst_HeartBeat( void )
{
  return analytics.HrtBeat_Flg;
}

  /**
  * @brief  This function sets the Heart Beat Flag.
  * @param  None
  * @retval None
  */
void Set_HeartBeat( void )
{
  analytics.HrtBeat_Flg = true;
}

  /**
  * @brief  This function clears the Heart Beat Flag.
  * @param  None
  * @retval None
  */
void Clr_HeartBeat( void )
{
  analytics.HrtBeat_Flg = false;
}

  /**
  * @brief  This function resets the driver State List to all off.
  * @param  None
  * @retval None
  */
void Reset_DriverStates( void )
{
  driver_list.IMUSense = DRIVER_OFF;
  driver_list.Pressure = DRIVER_OFF;
  driver_list.Irradiance = DRIVER_OFF;
  driver_list.I2CState = DRIVER_OFF;
}

  /**
  * @brief  This function updates the Driver list based on parameter passed.
  * @param  task_defs Task: Driver State to be modified, bool New State for Driver
  * @retval None
  */
void Set_DriverStates( task_defs Task, bool State )
{
  switch(Task)
  {
  case IMU_STATE_TASK:
    driver_list.IMUSense = State;
    break;
  case IRRADIANCE_MNTR_TASK:
    driver_list.Irradiance = State;
    break;
  case PRESSURE_MNTR_TASK:
    driver_list.Pressure = State;
    break;
  case I2C_STATE:
    driver_list.I2CState = State;
    break;
  default:
    break;
  }
}

  /**
  * @brief  This function returns the status based on parameter passed.
  * @param  task_defs Task: Driver State to be modified
  * @retval bool State for Driver
  */
bool Get_DriverStates( task_defs Task )
{
  switch(Task)
  {
  case IMU_STATE_TASK:
    return driver_list.IMUSense;
    break;
  case IRRADIANCE_MNTR_TASK:
    return driver_list.Irradiance;
    break;
  case PRESSURE_MNTR_TASK:
    return driver_list.Pressure;
    break;
  case I2C_STATE:
    return driver_list.I2CState;
    break;
  default:
    return DRIVER_OFF;
    break;
  }
}

  /**
  * @brief  This function returns the status of all Drivers.
  * @param  task_defs Task: Driver State to be modified
  * @retval bool State for Driver
  */
uint16_t Get_DriverStatus( void )
{
  uint16_t Status = 0x00;
  
  if ( Get_DriverStates( IMU_STATE_TASK ) )
    Status += 0x0001;
  if ( Get_DriverStates( IRRADIANCE_MNTR_TASK ) )
    Status += 0x0002;
  if ( Get_DriverStates( PRESSURE_MNTR_TASK ) )
    Status += 0x0004;
  if ( Get_DriverStates( I2C_STATE ) )
    Status += 0x0008;
  return Status;
}

