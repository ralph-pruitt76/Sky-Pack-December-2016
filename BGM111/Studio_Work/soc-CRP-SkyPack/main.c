/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "aat.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Device initialization header */
#include "InitDevice.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */

#include "main.h"
#include "uart_echo.h"
#include "uartdrv.h"
#include <stdio.h>
#include <string.h>

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif

/* Gecko configuration parameters (see gecko_configuration.h) */
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = 0,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
  #ifdef FEATURE_PTI_SUPPORT
  .pti = &ptiInit,
  #endif
};

// Frm State
typedef enum
{
  NULL_PT		= 0x00,			// NULL		Init State No Code Yet
  FRONT_TAG		= 0x01,			// <		First Tag Found.
  FRWRD_SLASH	= 0x02,			// /		Forward Slash Found.
  LTR_F			= 0x03,			// F		Letter F Found.
  LTR_R			= 0x04,			// R		Letter R Found.
  LTR_M			= 0x05,			// M		Letter M Found.
}Frm_State;

// Tick State
typedef enum
{
  TK_NULL_PT		= 0x00,			// NULL		Init State No Code Yet
  TK_FRONT_TAG		= 0x01,			// <		First Tag Found.
  TK_FRWRD_SLASH	= 0x02,			// /		Forward Slash Found.
  TK_LTR_T			= 0x03,			// T		Letter F Found.
  TK_LTR_I			= 0x04,			// I		Letter R Found.
  TK_LTR_C			= 0x05,			// C		Letter M Found.
  TK_LTR_K			= 0x06,			// K		Letter M Found.
//  TK_BACK_TAG		= 0x07,			// >		Last Tag Found.
}Tick_State;


/* MSG LOGGER */
struct
{
	char string[BUFFSIZE];;
	uint16 tx_wr;
	uint16 tx_rd;
	uint16 WrtBffr_Errcnt;
} static tx_Buffr;

struct
{
	char string[BUFFSIZE];;
	uint16 tx_wr;
	uint16 tx_rd;
} static rx_Buffr;

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;
int ticker = 0;
int connected;

// Key Flag for Tracking Read Buffer Overflow.
static bool Read_bufferOverflow;

/* Next buffer index based on current index and buffer size */

uint16_t NextBufIdxIncr(uint16_t idx,uint16_t incr_cnt)
{
	int x;
	for (x=0; x<incr_cnt; x++)
	{
		idx++;
		if (idx>= BUFFSIZE)
			idx=0;
	}
	return idx;
}

/* Report if the buffer is full based on its indexes */

bool IsBufFull(uint16_t wr_idx, uint16_t rd_idx)
{
  return NextBufIdxIncr(wr_idx, 1) == rd_idx;
}

/* Get the used space in the buffer based on its indexes */

uint16_t BufUsed(uint16_t wr_idx, uint16_t rd_idx)
{
  int size = (int)wr_idx - (int)rd_idx;
  if (size < 0)
  {
    size = BUFFSIZE + size;
  }
  return size;
}

/* Get the free space in the buffer based on its indexes */

uint16_t BufFree(uint16_t wr_idx, uint16_t rd_idx)
{
  return (BUFFSIZE - 1) - BufUsed(wr_idx, rd_idx);
}

/**
 * @brief  CallBack routine for RX Chars from UART.
 * @param  None.
 * @retval None
 */
void UART_rx_callbackLocal(void)
{

}

void reset_rdBuffer(void)
{
	int x;

	rx_Buffr.tx_wr = 0;
	rx_Buffr.tx_rd = 0;
	for (x=0; x< BUFFSIZE; x++)
		rx_Buffr.string[x] = 0x00;
}

void reset_wrtBuffer(void)
{
	int x;

	tx_Buffr.tx_wr = 0;
	tx_Buffr.tx_rd = 0;
	for (x=0; x< BUFFSIZE; x++)
		tx_Buffr.string[x] = 0x00;
}

bool Test_Tick( char test_char )
{
	static Tick_State Tick_Var= NULL_PT;
	bool save_logic = false;

	switch (Tick_Var)
	{
	case TK_NULL_PT:
		if (test_char == '<')
			Tick_Var = TK_FRONT_TAG;
		break;
	case TK_FRONT_TAG:
		if (test_char == '/')
			Tick_Var = TK_FRWRD_SLASH;
		else
			Tick_Var = TK_NULL_PT;
		break;
	case TK_FRWRD_SLASH:
		if (test_char == 'T')
			Tick_Var = TK_LTR_T;
		else
			Tick_Var = TK_NULL_PT;
		break;
	case TK_LTR_T:
		if (test_char == 'I')
			Tick_Var = TK_LTR_I;
		else
			Tick_Var = TK_NULL_PT;
		break;
	case TK_LTR_I:
		if (test_char == 'C')
			Tick_Var = TK_LTR_C;
		else
			Tick_Var = TK_NULL_PT;
		break;
	case TK_LTR_C:
		if (test_char == 'K')
			Tick_Var = TK_LTR_K;
		else
			Tick_Var = TK_NULL_PT;
		break;
	case TK_LTR_K:
		Tick_Var = TK_NULL_PT;
		if (test_char == '>')
			save_logic = true;
		break;
	} // EndSwitch (Tick_Var)

	return save_logic;
}

bool Test_Frm( char test_char )
{
	static Frm_State Frm_Var= NULL_PT;
	bool save_logic = false;

	switch (Frm_Var)
	{
	case NULL_PT:
		if (test_char == '<')
			Frm_Var = FRONT_TAG;
		break;
	case FRONT_TAG:
		if (test_char == '/')
			Frm_Var = FRWRD_SLASH;
		else
			Frm_Var = NULL_PT;
		break;
	case FRWRD_SLASH:
		if (test_char == 'F')
			Frm_Var = LTR_F;
		else
			Frm_Var = NULL_PT;
		break;
	case LTR_F:
		if (test_char == 'R')
			Frm_Var = LTR_R;
		else
			Frm_Var = NULL_PT;
		break;
	case LTR_R:
		if (test_char == 'M')
			Frm_Var = LTR_M;
		else
			Frm_Var = NULL_PT;
		break;
	case LTR_M:
		Frm_Var = NULL_PT;
		if (test_char == '>')
			save_logic = true;
		break;
	} // EndSwitch (Frm_State)

	return save_logic;
}

/**
 * @brief  This function Sends the passed string out via the UART Channel 1.
 * @param  char *test_string: String to be sent via UART. Should be terminated by a NULL/0x00.
 * @retval None
 */
void UART_Send2(char *test_string)
{
	uint16_t nextidx;

// Test Code to force Read Error.
//	Read_bufferOverflow = true;

	while((*test_string != (char)0x00) &
			(!(Read_bufferOverflow)))
	{
		// Place into Buffer and update pointers.
		nextidx = NextBufIdxIncr(rx_Buffr.tx_wr, 1);
		if (nextidx != rx_Buffr.tx_rd)
		{
			/* Put the data in the buffer */
			rx_Buffr.string[rx_Buffr.tx_wr] = *test_string;
			/* Increment the write index */
			rx_Buffr.tx_wr = nextidx;
		}
		else
		{
			// Process Buffer Overflow.
			UART_Send( "RX OVERFLOW!\r\n" );
			// Reset Buffer.
			reset_rdBuffer();				// Reset Read Buffer.
			// Set Error Flag so Error will be reported at end of next frame.
			Read_bufferOverflow = true;
			break;							// Break Loop. Abandon Current string.
		}
		// Test UART for Character to Process.
//OLD
//		USART_Tx (USART1, *test_string);
		test_string++;
	}
}

/**
 * @brief  This function places the passed string into the Ouput BGM Buffer.
 * @param  char *test_string: String to be transferred. Should be terminated by a NULL/0x00.
 * @retval None
 */
void BGM_Send(char *test_string)
{
	char tempBffr2[40];
	uint16_t nextidx;
	int x;

	while(*test_string != (char)0x00)
	{
		nextidx = NextBufIdxIncr(tx_Buffr.tx_wr, 1);
	    if (nextidx != tx_Buffr.tx_rd)
//			    if (0)	// Test Code to force error.
	    {
	        /* Put the data in the buffer */
	    	tx_Buffr.string[tx_Buffr.tx_wr] = *test_string;
	        /* Increment the write index */
	    	tx_Buffr.tx_wr = nextidx;
	    	// Now Test for </FRM> Tag
	    } // EndIf (nextidx != tx_Buffr.tx_rd)
	    else
	    {
	    	// Process Buffer Overflow.
	    	reset_wrtBuffer();				// Reset Write Buffer.
	    	tx_Buffr.WrtBffr_Errcnt++;		// Increment Error Count.
			// Force new Msg for App
    		// Clear Buffer.
    		for (x=0; x<40; x++)
    			tempBffr2[x] = 0x00;
			sprintf( tempBffr2, "<X>WR_OVL</X>");
			//Send Msg back to Micro.
			UART_Send2( tempBffr2 );
			// Terminate with ? to allow Buffer to process.
			UART_Send2( "?" );
			// Place Msg into Write Buffer.
			sprintf( tx_Buffr.string, tempBffr2);
			tx_Buffr.tx_wr = NextBufIdxIncr(tx_Buffr.tx_wr, strlen(tempBffr2));
			// Now report the number of Errors.
			sprintf( tempBffr2, "Wrt_OV:%01d\r\n",  tx_Buffr.WrtBffr_Errcnt);
			UART_Send2( tempBffr2 );

	    	if( tx_Buffr.WrtBffr_Errcnt >= MAX_BBFRWRT_ERRS)
	    	{
				tx_Buffr.WrtBffr_Errcnt = 0;	// Set Error Cnt to zero.

				sprintf( tempBffr2, "OVERFLOW!\r\n" );
				UART_Send2( tempBffr2 );
	    	}
	    } // EndElse (nextidx != tx_Buffr.tx_rd)

	    // Increment Pointer to next Character in buffer.
		test_string++;
	} // EndWhile (*test_string != (char)0x00)
}

/**
 * @brief  background LED Timer Code.
 * @param  None.
 * @retval None
 */
void led_timer(void)
{
	// Ticker runs from 0 - 9
	if (ticker < 9)
		ticker++;
	else
		ticker = 0;

	// LED Blinking
	if (connected == 0)
	{
		// No connection -> drive LED using a "heart-beat" pattern: ON when ticker = 0 or 2, otherwise OFF
		if ((ticker == 0) || (ticker == 2))
		{

		}
		else
		{

		}
	} // EndIf (connected == 0)
	else
	{
		// Connected..Blink Fast
		if (ticker & 1)
		{

		}
		else
		{

		}
	} // EndElse(connected == 0)
}

/**
 * @brief  Main function
 */
int main(void)
{
	char string[STR_LEN];
	char tempBffr2[BUFFER_LNGTH];
	char FuelTag[BUFFER_LNGTH];
	bool Tick_Active = false;

	static bool Data_Active;

	uint8 conn_handle;

	int x;

	int	tx_num;						// Number of bytes in Buffer
	char rx_Byte;

	int max_len;
	int tx_size;
	int Percent_wrt;
	int Percent_rd;
	uint16_t nextidx;

	uint32 UART_Status;

//	int unrecoverable_error;

	struct gecko_msg_gatt_server_send_characteristic_notification_rsp_t* result_ptr;

	// Initialize Key Variables.
	Data_Active = false;
	Read_bufferOverflow = false;

	// Initialize Key Buffer Structure.
	reset_wrtBuffer();				// Reset Write Buffer.
	tx_Buffr.WrtBffr_Errcnt = 0;	// Set Error Cnt to zero.
	reset_rdBuffer();				// Reset Read Buffer.

	USART_Reset(USART1);

	/* Initialize peripherals */
	enter_DefaultMode_from_RESET();

	/* Initialize stack */
	gecko_init(&config);

	/* Uart Test*/
	//UART_Test();

	while (1) {
		// Test UART for Character to Process.
		UART_Status = USART_StatusGet( USART1 );
		if (UART_Status & UARTDRV_STATUS_RXDATAV)
		{
			// We Have data. time to process it.
			while ( UART_Status & UARTDRV_STATUS_RXDATAV )
			{
				// Read Byte from Channel
				rx_Byte = USART_RxDataGet( USART1 );
				// Place into Buffer and update pointers.
				nextidx = NextBufIdxIncr(tx_Buffr.tx_wr, 1);
			    if (nextidx != tx_Buffr.tx_rd)
//			    if (0)	// Test Code to force error.
			    {
			        /* Put the data in the buffer */
			    	tx_Buffr.string[tx_Buffr.tx_wr] = rx_Byte;
			        /* Increment the write index */
			    	tx_Buffr.tx_wr = nextidx;
			    	// Now Test for </FRM> Tag
			    	if (Test_Frm(rx_Byte))
			    	{
			    		//Test to See if we have processed a Fuel Tag.
			    		if (Tick_Active)
			    		{
			    			//Send FuelTag back to Micro.
			    			UART_Send2( FuelTag );
			    			// Terminate with ? to allow Buffer to process.
			    			UART_Send2( "?" );
			    			// Reset Flag.
			    			Tick_Active = false;
			    		}

			    		// We Found FRM Tag...Send Quick Msg back to Micro.
			    		// Ok...Clear Previous Buffer OVerflow count. We made it to a new FRM Tag.
			    		tx_Buffr.WrtBffr_Errcnt = 0;	// Set Error Cnt to zero.

						// Test for Read Buffer overflow
						if (Read_bufferOverflow)
						{
							// Yes It did Occur. Report it..
							// Clear Buffer.
				    		for (x=0; x<BUFFER_LNGTH; x++)
				    			tempBffr2[x] = 0x00;
				    		sprintf( tempBffr2, "<X>RD_OVL</X>");
							//Send Msg back to Micro.
							UART_Send2( tempBffr2 );
							// Terminate with ? to allow Buffer to process.
							UART_Send2( "?" );
							// Place msg into BGM Buffer;
							BGM_Send( tempBffr2 );
							// Clear Flag.
							Read_bufferOverflow = false;
						} // EndIf (Read_bufferOverflow)
			    	} // EndIf (Test_Frm(rx_Byte))

			    	// Now Test for </TICK> Tag
			    	if (Test_Tick(rx_Byte))
			    	{
			    		// We Found Tick Tag...Send Quick Msg back to Micro.
			    		// Get Percent Write and Percent Rd
			    		//Percent_wrt = (float)(((float)(BufFree(tx_Buffr.tx_wr, tx_Buffr.tx_rd))/ (float)BUFFSIZE)) * 100.0;
			    		Percent_wrt = (BufUsed(tx_Buffr.tx_wr, tx_Buffr.tx_rd)* 100)/ BUFFSIZE;
			    		Percent_rd = (BufUsed(rx_Buffr.tx_wr, rx_Buffr.tx_rd)* 100)/ BUFFSIZE;
			    		// Clear Buffer.
			    		for (x=0; x<BUFFER_LNGTH; x++)
			    			FuelTag[x] = 0x00;

						sprintf( FuelTag, "<X>SP%sR:%02dW:%02d</X>", VERSION_NUM, Percent_rd, Percent_wrt);
						// Place msg into BGM Buffer;
						BGM_Send( FuelTag );

						// Set Flag.
		    			Tick_Active = true;
			    	} // EndIf (Test_Frm(rx_Byte))

			    } // EndIf (nextidx != tx_Buffr.tx_rd)
			    else
			    {
			    	// Process Buffer Overflow.
			    	reset_wrtBuffer();				// Reset Write Buffer.
			    	tx_Buffr.WrtBffr_Errcnt++;		// Increment Error Count.
		    		// Clear Buffer.
		    		for (x=0; x<BUFFER_LNGTH; x++)
		    			tempBffr2[x] = 0x00;
					// Force new Msg for App
					sprintf( tempBffr2, "<X>WR_OVL</X>");
					//Send Msg back to Micro.
					UART_Send2( tempBffr2 );
					// Terminate with ? to allow Buffer to process.
					UART_Send2( "?" );
					// Place Msg into Write Buffer.
					sprintf( tx_Buffr.string, tempBffr2);
					tx_Buffr.tx_wr = NextBufIdxIncr(tx_Buffr.tx_wr, strlen(tempBffr2));
		    		// Clear Buffer.
		    		for (x=0; x<BUFFER_LNGTH; x++)
		    			tempBffr2[x] = 0x00;
					// Now report the number of Errors.
					sprintf( tempBffr2, "Wrt_OV:%01d\r\n",  tx_Buffr.WrtBffr_Errcnt);
					UART_Send2( tempBffr2 );

			    	if( tx_Buffr.WrtBffr_Errcnt >= MAX_BBFRWRT_ERRS)
			    	{
						tx_Buffr.WrtBffr_Errcnt = 0;	// Set Error Cnt to zero.

			    		// Clear Buffer.
			    		for (x=0; x<BUFFER_LNGTH; x++)
			    			tempBffr2[x] = 0x00;
						sprintf( tempBffr2, "OVERFLOW!\r\n" );
						UART_Send2( tempBffr2 );
			    	} // EndIf ( tx_Buffr.WrtBffr_Errcnt >= MAX_BBFRWRT_ERRS)
//			    	unrecoverable_error = 1;
			    } // EndElse (nextidx != tx_Buffr.tx_rd)
			    // Test UART for Character to Process.
			    UART_Status = USART_StatusGet( USART1 );
			} //  EndWhile (UART_Status & UARTDRV_STATUS_RXDATAV)
		} // EndIf (UART_Status & UARTDRV_STATUS_RXDATAV)

		// Test UART for TX Empty.
		if (UART_Status & UARTDRV_STATUS_TXBL)
		{
			// We TX Empty.
			while ((BufUsed(rx_Buffr.tx_wr, rx_Buffr.tx_rd) > 0) &&
					( UART_Status & UARTDRV_STATUS_TXBL ))
			{
					// Place Byte into USART.
					USART1->TXDATA = rx_Buffr.string[rx_Buffr.tx_rd];

					// Update Pointer.
					rx_Buffr.tx_rd = NextBufIdxIncr(rx_Buffr.tx_rd, 1);

					// Reload Status.
					UART_Status = USART_StatusGet( USART1 );
			} //  EndWhile ((BufUsed(rx_Buffr.tx_wr, rx_Buffr.tx_rd) > 0) && ( UART_Status & UARTDRV_STATUS_TXBL ))
		} // EndIf (BufUsed(rx_Buffr.tx_wr, rx_Buffr.tx_rd) > 0)

		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;

		/* Check for stack event. */
		//evt = gecko_wait_event();
		evt = gecko_peek_event();
		if (evt != 0)
		{
			/* Handle events */
			switch (BGLIB_MSG_ID(evt->header)) {
				/* This boot event is generated when the system boots up after reset.
				 * Here the system is set to start advertising immediately after boot procedure. */
				case gecko_evt_system_boot_id:

					// Send Key Msgs to UART Channel.
					UART_Send2( "SP SPP server..." );
					// Terminate with ? to allow Buffer to process.
					UART_Send2( "?" );
		    		// Clear Buffer.
		    		for (x=0; x<BUFFER_LNGTH; x++)
		    			tempBffr2[x] = 0x00;
					sprintf(tempBffr2, "%s\r\n", LEGACY_BANNER);
					UART_Send2( tempBffr2 );
					sprintf(tempBffr2, "Boot. Build number: %ld\r\n", evt->data.evt_dfu_boot.version);
					UART_Send2( tempBffr2 );

					/* Set advertising parameters. 100ms advertisement interval. All channels used.
					 * The first two parameters are minimum and maximum advertising interval, both in
					 * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
					gecko_cmd_le_gap_set_adv_parameters(160, 160, 7);

					/* Start general advertising and enable connections. */
					gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);

					// setup timer with ~10ms interval.
					gecko_cmd_hardware_set_soft_timer(328, 0, 0);

					// start 100ms timer for LED control.
					//gecko_cmd_hardware_set_soft_timer(3277, 1, 0);

					// Clear Data Acive Flag;
					Data_Active = false;
					break;

				case gecko_evt_gatt_server_attribute_value_id:
					// writes to SPP data characteristic are copied directly to UART.
					if ( evt->data.evt_gatt_server_attribute_value.attribute == gattdb_xgatt_spp_data)
					{
						// Clear string before using
						for (x=0; x<STR_LEN; x++ )
							string[x] = 0x00;
						// Copy String to local string.
						strncpy(string, (char *)evt->data.evt_gatt_server_attribute_value.value.data,
								evt->data.evt_gatt_server_attribute_value.value.len);
						// Send to Micro.
						UART_Send2( string );
					}
					break;

				case gecko_evt_gatt_server_characteristic_status_id:
					if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_xgatt_spp_data) &&
							(evt->data.evt_gatt_server_characteristic_status.status_flags & gatt_server_client_config ) &&
							(evt->data.evt_gatt_server_characteristic_status.client_config_flags & gatt_notification ) )
					{
						// Data Channel is now active. TIme to setup for processing of data.
						Data_Active = true;
			    		// Clear Buffer.
			    		for (x=0; x<BUFFER_LNGTH; x++)
			    			tempBffr2[x] = 0x00;
						sprintf( tempBffr2, "DATA N\r\n" );
						UART_Send2( tempBffr2 );
						// Set Mode Not discoverable/Undirected connectable(Comment out Next Line to allow Discoverable)
						// OK...We need to set a 3 Seconds Timer to wakeup once. Handle 1.
						gecko_cmd_hardware_set_soft_timer(98400, 1, 1);

						//gecko_cmd_le_gap_set_mode(le_gap_non_discoverable, le_gap_undirected_connectable);
					}
					break;

				case gecko_evt_le_connection_opened_id:
		    		// Clear Buffer.
		    		for (x=0; x<BUFFER_LNGTH; x++)
		    			tempBffr2[x] = 0x00;
					// Send Key Msgs to UART Channel.
					sprintf( tempBffr2, "Connected\r\n" );
					UART_Send2( tempBffr2 );

					connected = 1;

					// make copy of connection handle for later use.
					conn_handle = evt->data.evt_le_connection_opened.connection;
					// conn_handle...min_interval...max_interval...latency...timeout
					// min_interval = value * 1.25ms
					// max_interval = value * 1.25ms
					// latency: This parameter defines how many connection intervals the slave can skip if it has no data to send
					// timeout = value * 10ms
					gecko_cmd_le_connection_set_parameters(conn_handle, 6, 6, 0, 10);
					// Clear Data Acive Flag;
					Data_Active = false;
					break;

				case gecko_evt_le_connection_parameters_id:
		    		// Clear Buffer.
		    		for (x=0; x<BUFFER_LNGTH; x++)
		    			tempBffr2[x] = 0x00;
					sprintf( tempBffr2, "conn.interval %d\r\n", evt->data.evt_le_connection_parameters.interval );
					UART_Send2( tempBffr2 );
					break;

				case gecko_evt_le_connection_closed_id:
		    		// Clear Buffer.
		    		for (x=0; x<BUFFER_LNGTH; x++)
		    			tempBffr2[x] = 0x00;
					sprintf( tempBffr2, "Disconnected\r\n" );
					UART_Send2( tempBffr2 );

					connected = 0;
					conn_handle = 0xff;

					// Clear Data Acive Flag;
					Data_Active = false;

					/* Check if need to boot to dfu mode */
					if (boot_to_dfu) {
						/* Enter to DFU OTA mode */
						gecko_cmd_system_reset(2);
					} else {
						/* Restart advertising after client has disconnected */
						gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
					}
					break;

				case gecko_evt_hardware_soft_timer_id:
					if (evt->data.evt_hardware_soft_timer.handle == 1)
					{
						// OK....Let's Turn off Advertising...
						gecko_cmd_le_gap_set_mode(le_gap_non_discoverable, le_gap_undirected_connectable);
						break;
					}

					// If Data Channel is active, Try to send Data.
					if (Data_Active)
					{
						while (BufUsed(tx_Buffr.tx_wr, tx_Buffr.tx_rd) > 0)
						{
							tx_num = BufUsed(tx_Buffr.tx_wr, tx_Buffr.tx_rd);
							// Send Data from the TX Buffer.
							if (tx_num > 20)
								tx_size = 20;
							else
								tx_size = tx_num;

							if (tx_size == 0)
								break;

							// max number of bytes that can be read from FIFO before wrap around
							max_len = BUFFSIZE - tx_Buffr.tx_rd;

							// this could be optimized for speed? now optimizing for simplicity..
							if (max_len < tx_size)
								tx_size = max_len;

							// Clear string before using
							for (x=0; x<STR_LEN; x++ )
								string[x] = 0x00;

							// Copy data from buffer to temp string and then send.
							strncpy(string, &tx_Buffr.string[tx_Buffr.tx_rd], tx_size);

							// call gatt_server_send_characteristic_notification($ff, xgatt_spp_data, tx_size, tx_fifo(tx_rd:tx_size))(res)
							result_ptr = gecko_cmd_gatt_server_send_characteristic_notification(
									conn_handle,
									gattdb_xgatt_spp_data,
									tx_size,
									(uint8 *)string);

							// Test Code : echo data to console.
							//sprintf( tempBffr2, "TX: %s\r\n", string );
							//UART_Send2( tempBffr2 );

							if (result_ptr->result != 0)
							{
					    		// Clear Buffer.
					    		for (x=0; x<BUFFER_LNGTH; x++)
					    			tempBffr2[x] = 0x00;
								sprintf( tempBffr2, "Write BLE Error: %04x\r\n", result_ptr->result );
								UART_Send2( tempBffr2 );
								// Bad Transfer...Shutdown Channel...
								Data_Active = false;
							}
							else
							{
								tx_Buffr.tx_rd = NextBufIdxIncr(tx_Buffr.tx_rd, tx_size);
							}
						} // EndWhile (tx_num > 20)
					} // EndIf (Data_Active)

					break;

				/* Events related to OTA upgrading
				 ----------------------------------------------------------------------------- */

					/* Check if the user-type OTA Control Characteristic was written.
					 * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
				case gecko_evt_gatt_server_user_write_request_id:

					if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
						/* Set flag to enter to OTA mode */
						boot_to_dfu = 1;
						/* Send response to Write Request */
						gecko_cmd_gatt_server_send_user_write_response(
								evt->data.evt_gatt_server_user_write_request.connection,
								gattdb_ota_control,
								bg_err_success);

						/* Close connection to enter to DFU OTA mode */
						gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
					}
					break;

				default:
		    		// Clear Buffer.
		    		for (x=0; x<BUFFER_LNGTH; x++)
		    			tempBffr2[x] = 0x00;
					sprintf(tempBffr2, "UNKNOWN EVT %lx\r\n", BGLIB_MSG_ID(evt->header));
					UART_Send2( tempBffr2 );
					break;
			} // EndSwitch (BGLIB_MSG_ID(evt->header))
		} // EndIf (evt != 0) ...If Valid Event..Handle.
	} //EndWhile (1)
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
