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

/* Characteristic handles */
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

// Analytics Structure
struct
{
  bool  HrtBeat_Flg;
  uint16_t HrtBeat_Cnt;
} static analytics;

struct
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
} data;

/* Initialize the IMU sensors */

void InitIMUSensors(void)
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
  LSM6DS3_Init(&LSM6DS3_InitStructure);
  
  /* Set up the magnetometer init structure */
  LIS3MDL_InitStructure.M_OutputDataRate = LIS3MDL_M_DO_10;
  LIS3MDL_InitStructure.M_OperatingMode = LIS3MDL_M_MD_CONTINUOUS;
  LIS3MDL_InitStructure.M_FullScale = LIS3MDL_M_FS_4;
  LIS3MDL_InitStructure.M_XYOperativeMode = LIS3MDL_M_OM_UHP;
  
  /* Initialize the magnetometer */
  LIS3MDL_Init(&LIS3MDL_InitStructure);
}

/* Read the IMU sensor data
 * - Accelerometer data is in 1/1000 g
 * - Gyro data is in 1/10 dps
 * - Magnetometer data is in mGauss
 * All scaling done according to typical sensitivity values in datasheet */

void ReadIMUSensors(void)
{
  int i;
  union u3DVector vector;
  
  uint8_t id = 0;
  LSM6DS3_Read_XG_ID(&id);
  
  /* Read the accelerometer data */
  if (LSM6DS3_X_GetAxesRaw(vector.indexed) == IMU_6AXES_OK)
  {
    /* Scale the values so they are in 1/1000 g when we run the sensor
     * at 4 g full scale (0.122 mg/LSB) */
    for (i=0; i<3; i++)
    {
      data.imu.accel.indexed[i] = (int16_t)
                        (((int32_t)(vector.indexed[i]) * 4000) / 32786);
    }
  }
  
  /* Read the gyro data */
  if (LSM6DS3_G_GetAxesRaw(vector.indexed) == IMU_6AXES_OK)
  {
    /* Scale the values so they are in 1/10 dps when we run the sensor
     * at 500 dps full scale (17.5 mdps/LSB) */
    for (i=0; i<3; i++)
    {
      data.imu.gyro.indexed[i] = (int16_t)
                        (((int32_t)(vector.indexed[i]) * 5000) / 28571);
    }
  }
  
  /* Read the magnetometer data */
  if (LIS3MDL_M_GetAxesRaw(vector.indexed) == MAGNETO_OK)
  {
    /* Scale the values so they are in mgauss when we run the sensor
     * at 4 gauss full scale (6842 LSB/gauss) */
    for (i=0; i<3; i++)
    {
      data.imu.mag.indexed[i] = (int16_t)
                        (((int32_t)(vector.indexed[i]) * 4000) / (6842 * 4));
    }
  }
}

/* Initialize the pressure/temperature sensor */

void InitPressureTempSensor(void)
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
  LPS25HB_Init(&LPS25HB_InitStructure);
  
  /* Set up the separate temperature sensor init structure */
  PCT2075_InitStructure.Device_Mode = PCT2075_DEVMODE_NORMAL;
  PCT2075_InitStructure.OS_Queue = PCT2075_OSFQUEUE_1;
  PCT2075_InitStructure.OS_Polarity = PCT2075_OSPOL_LOW;
  PCT2075_InitStructure.OS_Mode = PCT2075_OSMODE_COMP;
  PCT2075_InitStructure.OS_Trigger = 0.0f;
  PCT2075_InitStructure.OS_Hyst = 0.0f;
  PCT2075_InitStructure.Idle_Time = 2;
  
  /* Initialize the separate temperature sensor */
  PCT2075_Init(&PCT2075_InitStructure);
}

/* Read the pressure and temperature data
 * - Pressure is in 1/10 mbar
 * - Temperature is in 1/10 degree celcius */

void ReadPressureTempSensors(void)
{
  float value;
  
  /* Read the temperature */
  if (LPS25HB_GetTemperature(&value) == PRESSURE_OK)
  {
    /* Scale the value so it is in 1/10 C */
    data.temperature = (int16_t)(value * 10);
  }
  else
  {
    /* If that failed, try the separate temperature sensor */
    data.temperature = (int16_t)(PCT2075_GetTemp() * 10);
  }
  
  /* Read the pressure */
  if (LPS25HB_GetPressure(&value) == PRESSURE_OK)
  {
    /* Scale the value so it is in 1/10 mbar */
    data.pressure = (int16_t)(value * 10);
  }
}

/* Initialize the irradiance sensor */

void InitIrradianceSensor(void)
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
  OPT3001_Init(&OPT3001_InitStructure);
}

/* Initialize sample timer */

void InitSampleTimer(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable clock for the sample timer */
  RCC_APB1PeriphClockCmd(SAMPLE_TIM_RCC, ENABLE);

  /* Configure to generate an interrupt every 200ms */
  TIM_TimeBaseInitStructure.TIM_Prescaler = 32 * 1000;
  TIM_TimeBaseInitStructure.TIM_Period = 200;
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
  data.cap.event_freq = GetCapSenseEventFreq();
  data.cap.swept_idx = GetSweptFreqHighIdx();
  data.cap.swept_level = GetSweptFreqHighLevel();
}

/* Initialize all sensors */

void InitSensors(void)
{
  data.Legacy_OneTime = true;                   // Clear Legacy One time flag so that we can set key characteristics...once.
  /* Initialize the I2C peripheral */
  I2C_LowLevel_Init();
  
  /* Initialize I2C connected sensors for the application */
  InitIMUSensors();
  InitPressureTempSensor();
  InitIrradianceSensor();
  
  /* Initialize misc sensors we run from the micro */
  MiscSensors_Init();
  
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
}

/* Process sensor state machine */

void ProcessSensorState(void)
{
  char characteristic[21];
  uint8_t tempStr[13];
  static int nullCnt = 0;
  
  /* Is a reading scheduled? */
  if (data.reading_scheduled)
  {
    // Test Connection. Have we timed out??
    Test_Connection();
    
    /* Clear the scheduling flag */
    data.reading_scheduled = false;
    
    /* Read the IMU sensor data */
    ReadIMUSensors();
    /* Read the pressure and temperature */
    ReadPressureTempSensors();
    /* Read the irradiance in 1/100 lux */
    data.irradiance = OPT3001_GetData();
    /* Read the miscellaneous cap sensors */
    ReadMiscSensors();
    
    // Verify Data quality by checking irradiance.
    if ( data.irradiance == 0)
    {
      // We have null data from irradiance...Inc nullCnt.
      nullCnt++;
      if (nullCnt >= NULL_MAX)
      {
        nullCnt = 0;
        // Oops...Detected a fatal error...RESET!!!
        SkyPack_Reset( FATAL_I2CDROP );
      }
    }
    
    /* Create the accelerometer characteristic string */
    sprintf(characteristic, "#6%+05d%+05d%+05d", data.imu.accel.named.x,
            data.imu.accel.named.y, data.imu.accel.named.z);
    /* Send the accelerometer data to the BLE module */
    BGM111_WriteCharacteristic(gattdb_accelerometer,
                            strlen(characteristic), (uint8_t *)characteristic);
    /* Create the gyro characteristic string */
    sprintf(characteristic, "#9%+05d%+05d%+05d", data.imu.gyro.named.x,
            data.imu.gyro.named.y, data.imu.gyro.named.z);
    /* Send the gyro data to the BLE module */
    BGM111_WriteCharacteristic(gattdb_gyro,
                            strlen(characteristic), (uint8_t *)characteristic);
    /* Create the magnetometer characteristic string */
    sprintf(characteristic, "#7%+05d%+05d%+05d", data.imu.mag.named.x,
            data.imu.mag.named.y, data.imu.mag.named.z);
    /* Send the magnetometer data to the BLE module */
    BGM111_WriteCharacteristic(gattdb_magnetometer,
                            strlen(characteristic), (uint8_t *)characteristic);

    /* Create the temperature characteristic string */
    sprintf(characteristic, "#8%+05d", data.temperature);
    /* Send the temperature to the BLE module */
    BGM111_WriteCharacteristic(gattdb_temperature,
                            strlen(characteristic), (uint8_t *)characteristic);
    /* Create the pressure characteristic string */
    sprintf(characteristic, "#E%+06d", data.pressure);
    /* Send the pressure to the BLE module */
    BGM111_WriteCharacteristic(gattdb_pressure,
                            strlen(characteristic), (uint8_t *)characteristic);

    /* Create the irradiance characteristic string */
    sprintf(characteristic, "#B%+08d", data.irradiance);
    /* Send the irradiance to the BLE module */
    BGM111_WriteCharacteristic(gattdb_irradiance,
                            strlen(characteristic), (uint8_t *)characteristic);
    
    /* Create cap sense event frequency characteristic string */
    sprintf(characteristic, "#C%+05d", data.cap.event_freq);
    /* Send the cap sense event frequency to the BLE module */
    BGM111_WriteCharacteristic(gattdb_wiper_freq,
                            strlen(characteristic), (uint8_t *)characteristic);
    /* Create swept frequency characteristic string */
    sprintf(characteristic, "#D%+05d%+06d", data.cap.swept_idx,
                                            data.cap.swept_level);
    /* Send the swept frequency to the BLE module */
    BGM111_WriteCharacteristic(gattdb_swept_freq_idx,
                            strlen(characteristic), (uint8_t *)characteristic);
    // Test Analytics flag and determine if we need to update that characteristic
    if (!(Tst_HeartBeat()))
    {
      sprintf( (char *)tempStr, "%010dHB", analytics.HrtBeat_Cnt++);
      BGM111_WriteCharacteristic(gattdb_AnlHrtBt,
                             strlen((char *)tempStr), (uint8_t *)tempStr);
      Set_HeartBeat();
    }
    // Test Legacy flag to perform one time operations
    if( data.Legacy_OneTime )
    {
      // Clear Flag...We are done.
      data.Legacy_OneTime = false;
      // Update BLE Characteristics
      BGM111_WriteCharacteristic(gattdb_xgatt_rev,
                                 strlen((char *)LEGACY_BANNER), (uint8_t *)LEGACY_BANNER);
    }
  }
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
  SetLED(true);
  delay_10ms();
  SetLED(false);
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
  
  // Alert User by Blinking LED Fast and then wait 1 Second.
  for (x=0; x<code; x++)
  {
    SetLED(true);
    delay_100msec(1);
    SetLED(false);
    delay_100msec(1);
  }
  SetLED(true);
  delay_100msec(10);
  SetLED(false);
  delay_100msec(10);
  // Alternate way to reset....Power Down Power Plane.
  SetUSBPower( USB_POWER_OFF );
  delay_100msec(100);
  
  // Reset Micro and Start Over...
  NVIC_SystemReset();
}

  /**
  * @brief  This function Tests for an active connection.
  * @param  None
  * @retval None
  */
void Test_Connection( void )
{
  static uint16_t connection_cnt = 0;
  static uint16_t HeartBeat_Cnt = 0;
 
  // Test Connection
  if ( BGM111_Connected() )
  {
    // Yes...Clear count
    connection_cnt = 0;
     // Test Heart Beat. Has it been cleared?
    if (Tst_HeartBeat())
    {
      // No We need to watch this closely.
      // Test Heart Beat Count and determine if time to reset.
      HeartBeat_Cnt++;
      // Test Heart Beat Count. If expired, reset.
      if (HeartBeat_Cnt > HEARTBEAT_CNT)
      {
        // Has been 30 Seconds....Time to reset Code.
        HeartBeat_Cnt = 0;
        Clr_HrtBeat_Cnt();
        SkyPack_Reset( FATAL_TIMEOUT );
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
    
    // Wait 90 Seconds before forcing reset.
    if (connection_cnt > CONNECTION_CNT)
    {
      Clr_HrtBeat_Cnt();
      SkyPack_Reset( FATAL_TIMEOUT );
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

