#include "sys_ctrl.h"
#include "bgm111.h"
#include "app_data.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "miscRoutines.h"
#include "i2c_bus.h"
#include "ErrCodes.h"
#include "wwdg.h"
#include "parser.h"
#include "Calibration.h"

/* Program entry point */

void main()
{
  HAL_StatusTypeDef Status;
  uint8_t tempBffr2[80];
  uint8_t mntrCmd[10];

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
  
  //**
  //**
  //** Initialize all Flash Structures.
  //**
  //**
  //*******1. Initializ WWDG Flash Structure
  // 1a. Is WWDG Flash Frame Initialized?
#ifdef STM32L151CBT6
  if (SkyBrd_WWDG_VerifyFrame())
  {
    //Yes....Set FRAME_TASK Bit in Driver State Variable.
    Set_DriverStates( FRAME_TASK, DRIVER_ON );
  } // EndIf (RoadBrd_WWDG_VerifyFrame())
  else
  {
    //No....1b. Attempt to Initialize WWDG Flash Frame.
    if (SkyBrd_WWDG_InitializeFrmFlash() != HAL_OK)
    {
      //FAILED....Indicate Error Code and Fail Driver State.
      SkPck_ErrCdLogErrCd( ERROR_FRAME_INIT, MODULE_main );
      Set_DriverStates( FRAME_TASK, DRIVER_OFF );
    }
    else
    {
      //SUCCESS....Set FRAME_TASK Bit in Driver State Variable.
      Set_DriverStates( FRAME_TASK, DRIVER_ON );
    }
  } // EndElse (SkyBrd_WWDG_VerifyFrame())
  //*******2. Initializ Calibration Flash Structure
  // 2a. Is Calibration Flash Frame Initialized?
  if (SkyPack_CAL_VerifyFrame())
  {
    //Yes....Set CAL_TASK Bit in Driver State Variable.
    Set_DriverStates( CAL_TASK, DRIVER_ON );
  } // EndIf (RoadBrd_WWDG_VerifyFrame())
  else
  {
    //No....2b. Attempt to Initialize Structure Flash Structure.
    if (SkyPack_CAL_InitializeFrmFlash() != HAL_OK)
    {
      //FAILED....Indicate Error Code and Fail Driver State.
      SkPck_ErrCdLogErrCd( ERROR_CAL_INIT, MODULE_main );
      Set_DriverStates( CAL_TASK, DRIVER_OFF );
    }
    else
    {
      //SUCCESS....Set FRAME_TASK Bit in Driver State Variable.
      Set_DriverStates( CAL_TASK, DRIVER_ON );
    }
  } // EndElse (RoadBrd_WWDG_VerifyFrame())
#endif

  // Test I2C Status and Task init I2C if Active driver.
  if ( Get_DriverStates( I2C_STATE ) )
  {
    InitSensors();
  }

  // Display Banner
  strcpy( (char *)tempBffr2, "*********************  WEATHERCLOUD *********************\r\n\r\n");
  Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
  if (Status != HAL_OK)
    SkyPack_Reset( FATAL_ERROR );
  sprintf( (char *)tempBffr2, "     Sky Pack Monitor %s Hardware Version %s \r\n", VERSION_NUM, BRD_REV);
  Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
  if (Status != HAL_OK)
    SkyPack_Reset( FATAL_ERROR );;
  sprintf( (char *)tempBffr2, "                  Copyright %s. \r\n\r\n", REL_DATE);
  Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
  if (Status != HAL_OK)
    SkyPack_Reset( FATAL_ERROR );;
#ifdef STM32L151CBT6
    sprintf( (char *)tempBffr2, "Sensor Sample Rate: %3.1f Seconds.\r\n", ((float)SkyPack_GetSampleTime()/10));
    Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
    if (Status != HAL_OK)
      SkyPack_Reset( FATAL_ERROR );
    // Now Display the Units Enabled State.
    if (SkyPack_Get_UnitsFlag())
    {
      sprintf( (char *)tempBffr2, "Units XML State: ENABLED\r\n\r\n> ");
    }
    else
    {
      sprintf( (char *)tempBffr2, "Units XML State: DISABLED\r\n\r\n> ");
    }
    Status = SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
    if (Status != HAL_OK)
      SkyPack_Reset( FATAL_ERROR );
#endif
  
  BGM111_Init();

  /* Endless main loop */
  for (;;)
  {
    /* Process BLE input */
    BGM111_ProcessInput();
    /* Process the sensor state machine if the BLE module is ready */
    if ((BGM111_Ready()) &&
        (BGM111_Connected()) &&
        (BGM111_DataConnected()) &&
        (BGM111_SyncModeTestNoInc()) )
    {
      // Service Watchdog
//**CHANGE**
//      RoadBrd_WWDG_Refresh();     // Refresh WatchDog
      ProcessSensorState();
    }
    // Do we have a command to parse?
    if ( Mntr_Cmd() )
    {
      // Parse Monitor Command.
      sprintf( (char *)tempBffr2, "\r\n\r\n*********MONITOR COMMAND*********\r\n\r\n> ");
      SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
      getMntrCmd( mntrCmd );
      sprintf( (char *)tempBffr2, "CMD Received<%s>\r\n\r\n", mntrCmd);
      SkyPack_MNTR_UART_Transmit( (uint8_t *)tempBffr2 );
      // We have a good Tasking String. Time to determine action.
      Status = SkyBrd_ParseString((char *)mntrCmd);
      if (Status != HAL_OK)
        SkyPack_Reset( FATAL_ERROR );
      //Mntr_Clr();
    }
    /* Sleep when we have nothing to process */
    //PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
  }
}

