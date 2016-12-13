/* Miscellaneous sensors header file
 * Drivers for the extra sensors on the board that are managed by
 * microcontroller peripherals such as timers, comparator, ADC, ... :
 * wiper and precipitation type sensors. */

#ifndef MISC_SENSE_H_
#define MISC_SENSE_H_

#include "stm32l1xx.h"


/* Pin definitions */

#define CAPOUT_PORT             GPIOA
#define CAPOUT_PIN              GPIO_Pin_2
#define CAPOUT_RCC              RCC_AHBPeriph_GPIOA

#define CAPINCMP_PORT           GPIOB
#define CAPINCMP_PIN            GPIO_Pin_5
#define CAPINCMP_AF             GPIO_AF_RI
#define CAPINCMP_RCC            RCC_AHBPeriph_GPIOB
#define CAPINCMP_RCC_SOURCE     GPIO_PinSource5
#define CAPINCMP_TIM_CHAN       TIM_Channel_3   /* Remapped to TI4 */

#define CAPINRST_PORT           GPIOA
#define CAPINRST_PIN            GPIO_Pin_3
#define CAPINRST_AF             GPIO_AF_TIM2
#define CAPINRST_RCC            RCC_AHBPeriph_GPIOA
#define CAPINRST_RCC_SOURCE     GPIO_PinSource3
#define CAPINRST_TIM_CHAN       TIM_Channel_4

#define CAPDACCMP_PORT          GPIOA
#define CAPDACCMP_PIN           GPIO_Pin_4
#define CAPDACCMP_AF            GPIO_AF_RI
#define CAPDACCMP_RCC           RCC_AHBPeriph_GPIOA
#define CAPDACCMP_RCC_SOURCE    GPIO_PinSource4

#define SWEFRQOUT_PORT          GPIOB
#define SWEFRQOUT_PIN           GPIO_Pin_1
#define SWEFRQOUT_AF            GPIO_AF_TIM3
#define SWEFRQOUT_RCC           RCC_AHBPeriph_GPIOB
#define SWEFRQOUT_RCC_SOURCE    GPIO_PinSource1
#define SWEFRQOUT_TIM_CHAN      TIM_Channel_4

#define SWEFRQIN_PORT           GPIOB
#define SWEFRQIN_PIN            GPIO_Pin_0
#define SWEFRQIN_AF             GPIO_AF_RI
#define SWEFRQIN_RCC            RCC_AHBPeriph_GPIOB
#define SWEFRQIN_RCC_SOURCE     GPIO_PinSource0
#define SWEFRQIN_ADC_CHANNEL    ADC_Channel_8
#define SWEFRQIN_ADC_SAMPLE     ADC_SampleTime_96Cycles

/* Peripheral definitions */

#define SWEFRQIN_ADC            ADC1
#define SWEFRQIN_ADC_RCC        RCC_APB2Periph_ADC1

#define SWEFRQOUT_TIM           TIM3
#define SWEFRQOUT_TIM_RCC       RCC_APB1Periph_TIM3

#define SWEFRQMGR_TIM           TIM9
#define SWEFRQMGR_TIM_RCC       RCC_APB2Periph_TIM9
#define SWEFRQMGR_TIM_IRQn      TIM9_IRQn
#define SWEFRQMGR_TIM_IRQHandler TIM9_IRQHandler

#define CAP_TIM                 TIM2
#define CAP_TIM_RCC             RCC_APB1Periph_TIM2
#define CAP_TIM_IRQn            TIM2_IRQn
#define CAP_TIM_IRQHandler      TIM2_IRQHandler

#define CAPIN_COMP              COMP
#define CAPIN_COMP_RCC          RCC_APB1Periph_COMP

#define CAPDACCMP_DAC           DAC_Channel_1
#define CAPDACCMP_DAC_RCC       RCC_APB1Periph_DAC


/* Functions */

void MiscSensors_Init(void);
uint16_t GetCapSense(void);
uint16_t GetCapSenseBaseline(void);
uint16_t GetCapSenseEventFreq(void);
uint16_t GetSweptFreqHighIdx(void);
uint16_t GetSweptFreqHighLevel(void);


#endif
