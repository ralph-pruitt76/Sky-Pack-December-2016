/* Application data manager header file
 * Takes care of scheduling sample times and formatting the sensor data
 * to be sent as BLE characteristics to the app. */

#ifndef APP_DATA_H_
#define APP_DATA_H_

#include "stm32l1xx.h"


/* Sample timer definitions */

#define SAMPLE_TIM              TIM6
#define SAMPLE_TIM_RCC          RCC_APB1Periph_TIM6
#define SAMPLE_TIM_IRQn         TIM6_IRQn
#define SAMPLE_TIM_IRQHandler   TIM6_IRQHandler


/* Initialize all sensors */

void InitSensors(void);
   
/* Process sensor state machine */

void ProcessSensorState(void);


#endif