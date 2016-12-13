/* Miscellaneous sensors
 * Drivers for the extra sensors on the board that are managed by
 * microcontroller peripherals such as timers, comparator, ADC, ... :
 * wiper and precipitation type sensors. */

#include "misc_sense.h"
#include "sys_ctrl.h"


/* Cap sense timer interrupts per second */

#define CAP_TIMER_INT_PER_SEC           500UL

/* Cap sense comparator threshold (mV) */

#define CAP_SENSE_COMP_TH               900

/* Averaging filter shift constants */

#define CAP_AVG_FILTER_SHIFT            7
#define CAP_BASELINE_FILTER_SHIFT       8
#define CAP_EVENT_FILTER_SHIFT          2
#define SWEFRQ_FILTER_SHIFT             4

/* Cap sense thresholds */

#define CAP_SENSE_DETECT                16
#define CAP_SENSE_NO_DETECT             4
#define CAP_SENSE_DEBOUNCE              500UL

/* Cap sense average seconds */

#define CAP_SENSE_AVG_SEC               12

/* Swept frequency number of steps */

#define SWEFRQ_STEP_COUNT               150

/* Table of dividers for swept frequency sensor
 * The desired range of frequencies is 20kHz to 16MHz in 150 steps.
 * Ideally, the frequencies would be distributed evenly across the whole
 * frequency range, when viewed on a log scale.  However, at the high
 * end, we are limited by the dividers we have available, we can't take
 * steps as small as we'd like.  This table represents a compromise:
 * We use the minimum divider steps we can on the high end, but from the
 * moment we have enough resolution in the divider, we follow the linear
 * frequency distribution.
 * To see how this divider table was generated, refer to the "Swept freq
 * worksheet.ods" spreadsheet in the Documentation. */

static const uint16_t swept_freq_div_table[SWEFRQ_STEP_COUNT] =
{
  2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
  22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 39, 40,
  41, 43, 44, 46, 47, 49, 50, 52, 54, 56, 58, 59, 61, 64, 66, 68, 70, 73,
  75, 77, 80, 83, 86, 88, 91, 94, 98, 101, 104, 108, 111, 115, 119, 123,
  127, 131, 136, 140, 145, 150, 155, 160, 165, 171, 177, 183, 189, 195,
  202, 208, 215, 223, 230, 238, 246, 254, 262, 271, 280, 290, 299, 309,
  320, 330, 341, 353, 365, 377, 389, 402, 416, 430, 444, 459, 474, 490,
  506, 523, 541, 559, 578, 597, 617, 637, 659, 681, 704, 727, 751, 776,
  802, 829, 857, 886, 915, 946, 977, 1010, 1044, 1079, 1115, 1152, 1190,
  1230, 1271, 1314, 1357, 1403, 1450, 1498, 1548, 1600
};

/* Module globals to use for managing cap sense and swept
 * frequency subsystems */

static struct
{
  volatile uint32_t avg;
  volatile uint32_t baseline;
  uint8_t detect;
  uint16_t detect_debounce;
  volatile uint8_t events;
  uint8_t events_list[CAP_SENSE_AVG_SEC];
  uint8_t events_head;
  volatile uint32_t events_avg;
  uint16_t timer_int_div;
} cap_sense = { 0, 0, 0, 0, { 0 }, 0, 0, 0 };

static struct
{
  uint16_t idx;
  volatile uint32_t levels[SWEFRQ_STEP_COUNT];
  volatile uint16_t max_idx;
  volatile uint16_t max_level;
} swept_freq = { SWEFRQ_STEP_COUNT, { 0 }, 0, 0 };


/* Initialize the peripherals we will use to run our misc sensors */

void MiscSensors_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  COMP_InitTypeDef COMP_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  DAC_InitTypeDef DAC_InitStructure;
  ADC_InitTypeDef ACD_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  
  /* Enable clocks for the peripheral modules we'll use */
  RCC_APB1PeriphClockCmd(SWEFRQOUT_TIM_RCC | CAP_TIM_RCC |
                         CAPIN_COMP_RCC | CAPDACCMP_DAC_RCC, ENABLE);
  RCC_APB2PeriphClockCmd(SWEFRQIN_ADC_RCC | SWEFRQMGR_TIM_RCC, ENABLE);
  /* Enable GPIO clocks */
  RCC_AHBPeriphClockCmd(CAPOUT_RCC | CAPINCMP_RCC | CAPINRST_RCC |
                        SWEFRQOUT_RCC | SWEFRQIN_RCC | CAPDACCMP_RCC, ENABLE);
  /* The ADC uses the HSI clock, so turn it on */
  RCC_HSICmd(ENABLE);
  
  /* Set pin alternate functions */
  GPIO_PinAFConfig(CAPINCMP_PORT, CAPINCMP_RCC_SOURCE, CAPINCMP_AF);
  GPIO_PinAFConfig(CAPINRST_PORT, CAPINRST_RCC_SOURCE, CAPINRST_AF);
  GPIO_PinAFConfig(CAPDACCMP_PORT, CAPDACCMP_RCC_SOURCE, CAPDACCMP_AF);
  GPIO_PinAFConfig(SWEFRQOUT_PORT, SWEFRQOUT_RCC_SOURCE, SWEFRQOUT_AF);
  GPIO_PinAFConfig(SWEFRQIN_PORT, SWEFRQIN_RCC_SOURCE, SWEFRQIN_AF);

  /* Configure capacitive sensor output pin */
  GPIO_InitStructure.GPIO_Pin = CAPOUT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(CAPOUT_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(CAPOUT_PORT, CAPOUT_PIN, Bit_SET);

  /* Configure capacitive sensor input reset pin */
  GPIO_InitStructure.GPIO_Pin = CAPINRST_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(CAPINRST_PORT, &GPIO_InitStructure);
  
  /* Configure swept frequency sensor output pin */
  GPIO_InitStructure.GPIO_Pin = SWEFRQOUT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(SWEFRQOUT_PORT, &GPIO_InitStructure);
  
  /* Configure capacitive sensor input comparator pin as analog */
  GPIO_InitStructure.GPIO_Pin = CAPINCMP_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(CAPINCMP_PORT, &GPIO_InitStructure);
  
  /* Configure capacitive sensor DAC for comparator output pin
   * We actually don't use this pin (NC), but it seems when we turn on the
   * DAC to provide a ratiometric reference for the cap sense comparator,
   * this pin will automatically become a DAC output and we need to make
   * it an analog pin to prevent current leakage */
  GPIO_InitStructure.GPIO_Pin = CAPDACCMP_PIN;
  GPIO_Init(CAPDACCMP_PORT, &GPIO_InitStructure);
  
  /* Configure swept frequency sensor input pin */
  GPIO_InitStructure.GPIO_Pin = SWEFRQIN_PIN;
  GPIO_Init(SWEFRQIN_PORT, &GPIO_InitStructure);

  /* Configure TIM2 for use with the capacitive sensors:
   * Count up @ 32 MHz, 2 ms timer period */
  TIM_InternalClockConfig(CAP_TIM);
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInitStructure.TIM_Period = 32 * 2000;
  TIM_TimeBaseInit(CAP_TIM, &TIM_TimeBaseInitStructure);
  
  /* Configure capacitive sensor comparator input as timer capture input
   * on IC3 */
  TIM_ICInitStructure.TIM_Channel = CAPINCMP_TIM_CHAN;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
  TIM_ICInit(CAP_TIM, &TIM_ICInitStructure);
  
  /* Configure capacitive sensor reset input as timer compare output
   * on OC4 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_OPMode_Repetitive;
  TIM_OC4Init(CAP_TIM, &TIM_OCInitStructure);
  /* Discharge capacitor for 10 us */
  TIM_SetCompare4(CAP_TIM, 32 * 10);
  
  /* Configure the DAC to be used as a ratiometric reference for the
   * comparator inverting input */
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = 0;
  DAC_Init(CAPDACCMP_DAC, &DAC_InitStructure);
  DAC_Cmd(CAPDACCMP_DAC, ENABLE);
  /* Output a comparator reference */
  DAC_SetChannel1Data(DAC_Align_12b_R, 4095UL * CAP_SENSE_COMP_TH / 3300);
  
  /* Configure the COMP2 comparator to drive TIM2 input */
  COMP_InitStructure.COMP_Speed = COMP_Speed_Slow;
  COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_DAC1;
  COMP_InitStructure.COMP_OutputSelect = COMP_OutputSelect_TIM2IC4;
  COMP_Init(&COMP_InitStructure);
  
  /* Hook up the analog routing connections for COMP2 and ADC */
  SYSCFG_RIIOSwitchConfig(RI_IOSwitch_GR6_2, ENABLE);
  SYSCFG_RIIOSwitchConfig(RI_IOSwitch_CH8, ENABLE);
  
  /* Enable and configure TIM2 update interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = CAP_TIM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ITConfig(CAP_TIM, TIM_IT_Update, ENABLE);
  
  /* Enable TIM2 to start arradiance and cap sense systems */
  TIM_Cmd(CAP_TIM, ENABLE);

  /* Configure TIM3 to generate output for swept frequency sensor:
   * Count up @ 32 MHz */
  TIM_InternalClockConfig(SWEFRQOUT_TIM);
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInitStructure.TIM_Period = 0;
  TIM_TimeBaseInit(SWEFRQOUT_TIM, &TIM_TimeBaseInitStructure);

  /* Output compare for swept frequency output */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_OPMode_Repetitive;
  TIM_OC4Init(SWEFRQOUT_TIM, &TIM_OCInitStructure);
  
  /* Enable TIM3 to start generating swept frequency output */
  TIM_Cmd(SWEFRQOUT_TIM, ENABLE);

  /* Configure the ADC to convert the swept frequency sensor input */
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInit(&ADC_CommonInitStructure);
  ACD_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ACD_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ACD_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T9_CC2;
  ACD_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ACD_InitStructure.ADC_NbrOfConversion = 1;
  ACD_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ACD_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_Init(SWEFRQIN_ADC, &ACD_InitStructure);
  ADC_RegularChannelConfig(SWEFRQIN_ADC, SWEFRQIN_ADC_CHANNEL, 1,
                           SWEFRQIN_ADC_SAMPLE);
  ADC_Cmd(SWEFRQIN_ADC, ENABLE);
  /* Wait until the ADC and channel is ready */
  while(ADC_GetFlagStatus(SWEFRQIN_ADC, ADC_FLAG_ADONS) == RESET)
    ;
  while(ADC_GetFlagStatus(SWEFRQIN_ADC, ADC_FLAG_RCNR) == SET)
    ;
  
  /* Configure TIM9 to manage the swept frequency output and conversion:
   * Count up @ 16 MHz, 500us timer period */
  TIM_InternalClockConfig(SWEFRQMGR_TIM);
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 1;
  TIM_TimeBaseInitStructure.TIM_Period = 7999;
  TIM_TimeBaseInit(SWEFRQMGR_TIM, &TIM_TimeBaseInitStructure);

  /* Output compare to trigger swept frequency ADC conversion */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_OPMode_Repetitive;
  TIM_OC2Init(SWEFRQMGR_TIM, &TIM_OCInitStructure);
  /* Do ADC conversion after 480 us */
  TIM_SetCompare2(SWEFRQMGR_TIM, 16 * 480);

  /* Enable and configure TIM9 update interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = SWEFRQMGR_TIM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ITConfig(SWEFRQMGR_TIM, TIM_IT_Update, ENABLE);

  /* Enable TIM9 to start swept frequency manager */
  TIM_Cmd(SWEFRQMGR_TIM, ENABLE);
}

/* Cap sense interrupt handler */

void CAP_TIM_IRQHandler(void)
{
  /* Clear the interrupt flag */
  TIM_ClearITPendingBit(CAP_TIM, TIM_IT_Update);
  
  /* Get last cycle's timer capture value */
  uint16_t cap_tc = TIM_GetCapture3(CAP_TIM);
  
  /* Average cap sense */
  cap_sense.avg += cap_tc - (cap_sense.avg >> CAP_AVG_FILTER_SHIFT);
  /* Average cap sense baseline */
  cap_sense.baseline += (cap_sense.avg >> CAP_AVG_FILTER_SHIFT) - 
                  (cap_sense.baseline >> CAP_BASELINE_FILTER_SHIFT);

  /* Cap sense difference from baseline */
  int32_t cap_diff = (int32_t)(cap_sense.avg >> CAP_AVG_FILTER_SHIFT) -
                    (int32_t)(cap_sense.baseline >> CAP_BASELINE_FILTER_SHIFT);
  /* Was a previous cap sense event detected or not? */
  if (!cap_sense.detect)
  {
    /* No current event, are we above our detection threshold? */
    if (cap_diff > CAP_SENSE_DETECT)
    {
      /* We detected an event */
      cap_sense.detect = 1;
      cap_sense.detect_debounce = 0;
      cap_sense.events++;
      /* Pulse the LED */
      SetLED(true);
    }
  }
  else
  {
    /* We had a detected event, increment our debounce counter */
    cap_sense.detect_debounce++;
    /* Are we past the debounce period and did we fall back below our
     * detection threshold? */
    if (cap_sense.detect_debounce >=
        CAP_TIMER_INT_PER_SEC * CAP_SENSE_DEBOUNCE / 1000 && 
        cap_diff < CAP_SENSE_NO_DETECT)
    {
      /* No current event anymore */
      cap_sense.detect = 0;
      /* Turn the LED off */
      SetLED(false);
    }
  }
  
  /* Increment timer divider, did we reach a second? */
  if (++cap_sense.timer_int_div >= CAP_TIMER_INT_PER_SEC)
  {
    /* Reset the divider */
    cap_sense.timer_int_div = 0;
    /* Save the events for this second and clear the accumulator */
    cap_sense.events_list[cap_sense.events_head] = cap_sense.events;
    cap_sense.events = 0;
    /* Move the list index */
    if (++cap_sense.events_head >= CAP_SENSE_AVG_SEC)
      cap_sense.events_head = 0;
    /* Calculate the total number of events in the list */
    uint32_t total_events = 0;
    for (int i = 0; i < CAP_SENSE_AVG_SEC; i++)
      total_events += cap_sense.events_list[i];
    /* Update the events average frequency */
    cap_sense.events_avg += (total_events ? total_events :
                               cap_sense.events_avg ? -1 : 0) -
                            ((cap_sense.events_avg + 
                              (1 << (CAP_EVENT_FILTER_SHIFT-1)))
                             >> CAP_EVENT_FILTER_SHIFT);
  }
}

/* Swept frequency manager interrupt handler */

void SWEFRQMGR_TIM_IRQHandler(void)
{
  /* Clear the interrupt flag */
  TIM_ClearITPendingBit(SWEFRQMGR_TIM, TIM_IT_Update);
  
  /* Should we do an ADC reading? */
  if (swept_freq.idx < SWEFRQ_STEP_COUNT)
  {
    /* Read the ADC and store the reading in our list */
    swept_freq.levels[swept_freq.idx] += ADC_GetConversionValue(SWEFRQIN_ADC)
                - (swept_freq.levels[swept_freq.idx] >> SWEFRQ_FILTER_SHIFT);
  }
  
  /* Go to the next frequency step, did we reach the end? */
  if (++swept_freq.idx >= SWEFRQ_STEP_COUNT)
  {
    /* Wrap around if we reached the end */
    swept_freq.idx = 0;
    /* Find the index of the highest signal */
    uint32_t max_level = 0;
    for (int i = 0; i < SWEFRQ_STEP_COUNT; i++)
    {
      if (swept_freq.levels[i] > max_level)
      {
        max_level = swept_freq.levels[i];
        swept_freq.max_idx = i;
      }
    }
    /* Save the level of the highest signal */
    swept_freq.max_level = max_level;
  }
  
  /* Start outputting the new frequency by setting the output timer
   * divider and output capture */
  uint16_t new_div = swept_freq_div_table[swept_freq.idx];
  TIM_SetAutoreload(SWEFRQOUT_TIM, new_div - 1);
  TIM_SetCompare4(SWEFRQOUT_TIM, new_div >> 1);
  TIM_SetCounter(SWEFRQOUT_TIM, 0);
}

/* Get the capacitive sensor raw value */

uint16_t GetCapSense(void)
{
  return (uint16_t)(cap_sense.avg >> CAP_AVG_FILTER_SHIFT);
}

/* Get the capacitive sensor baseline */

uint16_t GetCapSenseBaseline(void)
{
  return (uint16_t)(cap_sense.baseline >> CAP_BASELINE_FILTER_SHIFT);
}

/* Get the cap sense event frequency (wiper frequency) in Hz / 100 */

uint16_t GetCapSenseEventFreq(void)
{
  return (uint16_t)((100UL * cap_sense.events_avg / CAP_SENSE_AVG_SEC)
                    >> CAP_EVENT_FILTER_SHIFT);
}

/* Get the swept frequency sensor frequency index with the highest
 * output level */

uint16_t GetSweptFreqHighIdx(void)
{
  return swept_freq.max_idx;
}

/* Get the swept frequency sensor highest level */

uint16_t GetSweptFreqHighLevel(void)
{
  return swept_freq.max_level;
}