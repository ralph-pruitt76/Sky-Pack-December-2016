/* System control
 * Drivers to control the capacitive sensing driven shield, the 
 * charging chip, the charging heater and the status LED */

#include "sys_ctrl.h"
#include "miscRoutines.h"

/* Initialize the system control */

void Sys_Ctrl_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
#ifdef LED_PWM
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
#endif
  
  /* Enable clocks for the peripheral modules we'll use */
  // Specifically this command enables RCC_APB2Periph_TIM11(TIM11EN/ TIM11 Timer Clock).
  RCC_APB2PeriphClockCmd(STATUS_LED_TIM_RCC, ENABLE);
  
  /* Enable GPIO clocks */
  // Specifically this command enables RCC_AHBPeriph_GPIOA and RCC_AHBPeriph_GPIOB.
  RCC_AHBPeriphClockCmd(SHIELD_ON_GPIO_CLK | CHARGE_ON_GPIO_CLK |
                         HEAT_ON_GPIO_CLK | STATUS_LED_GPIO_CLK, ENABLE);
  
  /* Set pin alternate functions */
  GPIO_PinAFConfig(STATUS_LED_GPIO_PORT, STATUS_LED_SOURCE, STATUS_LED_AF);
  
  /* SHIELD_ON, HEAT_ON pin configuration */
  GPIO_InitStructure.GPIO_Pin = SHIELD_ON_PIN | HEAT_ON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(SHIELD_ON_GPIO_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(SHIELD_ON_GPIO_PORT, SHIELD_ON_PIN | HEAT_ON_PIN);
  
  /* CHARGE_ON pin configuration */
  GPIO_InitStructure.GPIO_Pin = CHARGE_ON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(CHARGE_ON_GPIO_PORT, &GPIO_InitStructure);
  GPIO_SetBits(CHARGE_ON_GPIO_PORT, CHARGE_ON_PIN);
  
  // I2C_ON  pin configuration.
  GPIO_InitStructure.GPIO_Pin = VDD_ON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(VDD_ON_GPIO_PORT, &GPIO_InitStructure);
  SkyPack_gpio_On(gVDD_PWR);

  // BGM_ON  pin configuration.
  GPIO_InitStructure.GPIO_Pin = BGM_ON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(BGM_ON_GPIO_PORT, &GPIO_InitStructure);
  SkyPack_gpio_On(gBGM_PWR);
  
  /* BGM_LED configuration */
  GPIO_InitStructure.GPIO_Pin = STATUS_LED_PIN;
#ifdef LED_PWM
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
#else
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
#endif
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(STATUS_LED_GPIO_PORT, &GPIO_InitStructure);

  // STATUS_LED configuration.
  GPIO_InitStructure.GPIO_Pin = YELLOW_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(YELLOW_GPIO_PORT, &GPIO_InitStructure);
  SkyPack_gpio_Off(STATUS_LED);

  // MICRO_LED configuration.
  GPIO_InitStructure.GPIO_Pin = GREEN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GREEN_GPIO_PORT, &GPIO_InitStructure);
  SkyPack_gpio_Off(MICRO_LED);
  
#if LED_PWM
  /* Configure TIM11 to drive the status LED:
   * Count up @ 1kHz, 5 s timer period */
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 32000;
  TIM_TimeBaseInitStructure.TIM_Period = 5000;
  TIM_TimeBaseInit(STATUS_LED_TIM, &TIM_TimeBaseInitStructure);

  /* Configure output compare for pulsing the LED */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_OPMode_Repetitive;
  TIM_OC1Init(STATUS_LED_TIM, &TIM_OCInitStructure);
  /* Pulse 0.5 s out of 5 s */
  TIM_SetCompare1(STATUS_LED_TIM, 1);
  
  /* Start the status LED timer */
  TIM_Cmd(STATUS_LED_TIM, ENABLE);
#endif
}

/* Switch the USB power to off, battery charging or heating */

void SetUSBPower(enum eUSBPower power_setting)
{
  switch(power_setting)
  {
  default:
  case USB_POWER_OFF:
    GPIO_ResetBits(CHARGE_ON_GPIO_PORT, CHARGE_ON_PIN);
    GPIO_ResetBits(HEAT_ON_GPIO_PORT, HEAT_ON_PIN);
    break;
  case USB_POWER_CHARGE:
    GPIO_SetBits(CHARGE_ON_GPIO_PORT, CHARGE_ON_PIN);
    GPIO_ResetBits(HEAT_ON_GPIO_PORT, HEAT_ON_PIN);
    break;
  case USB_POWER_HEAT:
    GPIO_ResetBits(CHARGE_ON_GPIO_PORT, CHARGE_ON_PIN);
    GPIO_SetBits(HEAT_ON_GPIO_PORT, HEAT_ON_PIN);
    break;
  }
}

/* Switch capacitive sensing driven shield on or off */

void SetCapSenseShield(bool on)
{
  if (on)
    GPIO_SetBits(SHIELD_ON_GPIO_PORT, SHIELD_ON_PIN);
  else
    GPIO_ResetBits(SHIELD_ON_GPIO_PORT, SHIELD_ON_PIN);
}

#ifndef LED_PWM
/* Turn the LED on or off */

void SetLED(bool on)
{
  if (on)
    GPIO_SetBits(STATUS_LED_GPIO_PORT, STATUS_LED_PIN);
  else
    GPIO_ResetBits(STATUS_LED_GPIO_PORT, STATUS_LED_PIN);
}
#endif
