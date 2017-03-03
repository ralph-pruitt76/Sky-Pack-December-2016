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
  HAL_StatusTypeDef Status;
  uint8_t tempBffr2[80];

  /* Initialize all hardware */
  Sys_Ctrl_Init();
  // Initialize Monitor USART.
  MX_MNTR_UART_Init();

  SetCapSenseShield(true);
  BGM111_Init();
  InitSensors();

  // Display Banner
  strcpy( (char *)tempBffr2, "*********************  WEATHERCLOUD *********************\r\n\r\n");
  Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
  if (Status != HAL_OK)
    SkyPack_Reset( FATAL_ERROR );;
  sprintf( (char *)tempBffr2, "     Sky Pack Monitor %s Hardware Version %s \r\n", VERSION_NUM, BRD_REV);
  Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
  if (Status != HAL_OK)
    SkyPack_Reset( FATAL_ERROR );;
  sprintf( (char *)tempBffr2, "                  Copyright %s. \r\n\r\n\r\n> ", REL_DATE);
  Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
  if (Status != HAL_OK)
    SkyPack_Reset( FATAL_ERROR );;
 
  
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

