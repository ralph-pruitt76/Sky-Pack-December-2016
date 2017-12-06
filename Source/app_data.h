/* Application data manager header file
 * Takes care of scheduling sample times and formatting the sensor data
 * to be sent as BLE characteristics to the app. */

#ifndef APP_DATA_H_
#define APP_DATA_H_

#include "stm32l1xx.h"
#include "stdbool.h"
#include "bgm111.h"
#include "main.h"

typedef enum
{
  FATAL_SYNC		= 0x0002,	// Fatal Sync Error Detected.
  FATAL_PAYLDSYNC       = 0x0003,	// Fatal: payload bigger than 60 bytes, something's wrong.
  FATAL_OVERFLOW        = 0x0004,	// Fatal Buffer Overflow.
  FATAL_TIMEOUT		= 0x0005,	// 90 Second Timeout.
  FATAL_CNCTDROP	= 0x0006,	// Connection Dropped
  FATAL_I2CDROP         = 0x0007,       // I2C Channel Hung.
  FATAL_TXBGMBUF_FULL   = 0x0008,       // Transmit Buffer Full error on traffic to BGM111.
  FATAL_ERROR           = 0x0020,       // Generic Fatal Error(USART).
}ResetCodes;

/* Sample timer definitions */

#define SAMPLE_TIM              TIM6
#define SAMPLE_TIM_RCC          RCC_APB1Periph_TIM6
#define SAMPLE_TIM_IRQn         TIM6_IRQn
#define SAMPLE_TIM_IRQHandler   TIM6_IRQHandler
#define NULL_MAX                5        // Maximum number of null readings before reset.
#define ANALYTICS_MAXCNT        18       // 180 Seconds
#define PROCESS_SNSR_TIME       100      // Process all sensors every Second(100 * 100ms tick)...10.0 Seconds

#define FRM_REPEAT_CNT          5       // Repeat the full set of data at least this many times.
#define TICK_LIMIT              1       // Number Frames per Tick Event.
#define CANARY_CNT_LIMIT        1000000   // Loop 1,000,000 times through app loop without valid data good.

#ifdef LONG_DELAY
  #define CONNECTION_CNT          4500     // 15 Minutes.
  //#define HEARTBEAT_CNT           1500     // 5 Minutes.
  #define HEARTBEAT_CNT           900     // 3 Minutes.
#else
  #define CONNECTION_CNT          450      // 90 Seconds.
  #define HEARTBEAT_CNT           150      // 30 Seconds.
#endif


typedef enum 
{
  IMU_STATE_TASK = 0,
  IRRADIANCE_MNTR_TASK = 1,
  PRESSURE_MNTR_TASK = 2,
  I2C_STATE = 3,
  FRAME_TASK = 4,
  CAL_TASK = 5,
  TASK_LENGTH = 6
} task_defs;

typedef enum 
{
  DRIVER_OFF = 0,
  DRIVER_ON = 1
} driver_state;



/* Initialize all sensors */

void InitSensors(void);
void SetDataReady( void );
   
/* Process sensor state machine */

void ProcessSensorState(void);
void delay_100ms( void );
void delay_100msec( int value );
void delay_10ms( void );
void Flicker_Led( void );
void SkyPack_Reset( int code );
void Reset_DriverStates( void );
void Set_DriverStates( task_defs Task, bool State );
void SetTest_DriverStates( task_defs Task, bool State );
bool Get_DriverStates( task_defs Task );
uint16_t Get_DriverStatus( void );
void Test_Connection( void );
bool Tst_HeartBeat( void );
void Set_HeartBeat( void );
void Clr_HeartBeat( void );
void Clr_HrtBeat_Cnt( void );
void SendApp_String( uint8_t *pData );
void ClrDataStructure(void);
void ChangeSampleTimer(void);
void ClrAnalyticsRepeat( void );
bool TstDataReady( void );
void ClrDataReady( void );
void Clr_CMD_Md_Cnt( void );
void Update_TimeCnt( void );
uint32_t Read_TimeCnt( void );
char *getTickString( void );
void minimal_InitSensors(void);

#endif