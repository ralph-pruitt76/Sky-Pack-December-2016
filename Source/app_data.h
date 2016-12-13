/* Application data manager header file
 * Takes care of scheduling sample times and formatting the sensor data
 * to be sent as BLE characteristics to the app. */

#ifndef APP_DATA_H_
#define APP_DATA_H_

#include "stm32l1xx.h"

typedef enum
{
  FATAL_SYNC		= 0x0002,	// Fatal Sync Error Detected.
  FATAL_PAYLDSYNC       = 0x0003,	// Fatal: payload bigger than 60 bytes, something's wrong.
  FATAL_OVERFLOW        = 0x0004,	// Fatal Buffer Overflow.
  FATAL_TIMEOUT		= 0x0005,	// 90 Second Timeout.
  FATAL_CNCTDROP	= 0x0006,	// Connection Dropped
}ResetCodes;

/* Sample timer definitions */

#define SAMPLE_TIM              TIM6
#define SAMPLE_TIM_RCC          RCC_APB1Periph_TIM6
#define SAMPLE_TIM_IRQn         TIM6_IRQn
#define SAMPLE_TIM_IRQHandler   TIM6_IRQHandler


/* Initialize all sensors */

void InitSensors(void);
   
/* Process sensor state machine */

void ProcessSensorState(void);
void delay_100ms( void );
void delay_100msec( int value );
void SkyPack_Reset( int code );
void Test_Connection( void );

#endif