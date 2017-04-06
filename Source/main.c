#include "sys_ctrl.h"
#include "bgm111.h"
#include "app_data.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "miscRoutines.h"
#include "i2c_bus.h"

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

  // Test LEDs
  SkyPack_LEDTest();
  
  // Turn on Power Supplies.
  SkyPack_gpio_Off(gVDD_PWR);            // Turn on VDD(I2C Sensors).
  delay_100msec(2);                     // Wait 200 msec to settle.
  SkyPack_gpio_Off(gBGM_PWR);            // Turn on V+(BGM Power).

  // Wait for power to stabilize...200msec
  delay_100msec(2);
  // Reset all Drivers to Off before starting init process.
  Reset_DriverStates();

  // Test I2C Channel and see if we even have a working I2C.
  SkyPack_TestI2C();
  
  // Test I2C Status and Task init I2C if Active driver.
  if ( Get_DriverStates( I2C_STATE ) )
  {
    InitSensors();
  }

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
 
  BGM111_Init();

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
    //PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
  }
}

