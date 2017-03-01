#include "sys_ctrl.h"
#include "bgm111.h"
#include "app_data.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* Program entry point */

void main()
{
  /* Initialize all hardware */
  Sys_Ctrl_Init();
  // Initialize Monitor USART.
  MX_MNTR_UART_Init();

  SetCapSenseShield(true);
  BGM111_Init();
  InitSensors();
  
  /* Endless main loop */
  for (;;)
  {
    /* Process BLE input */
    BGM111_ProcessInput();
    /* Process the sensor state machine if the BLE module is ready */
    if (BGM111_Ready())
    {
      ProcessSensorState();
    }
    /* Sleep when we have nothing to process */
    PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
  }
}

