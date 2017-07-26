/* SiLabs BGM111 module access implementation using BGLib */

#include "bgm111.h"
#include <string.h>
#include <stdio.h>
#include "gecko_bglib.h"
#include "app_data.h"
#include "usart.h"
#include "ErrCodes.h"


/* BGLib instantiation */

BGLIB_DEFINE();

/* Define buffer size for BLE communication */

// #define BG_DATA_LENGTH          70    // Modified in ROad Board to 200. Same Overflow observed here 4/6/17.
#define BG_DATA_LENGTH          200   

/* Define number of free bytes in buffer when we need to process a packet
 * even if that means we need to wait in a loop */

#define BG_DATA_LOW_WATERMARK   5

/* BG reception states */

enum BgRxState
{
  BGRX_SYNC,
  BGRX_HDR,
  BGRX_DATA
};


/* MSG LOGGER */
struct
{
  uint32_t msgBuff[BG_DATA_LENGTH];
  uint16_t tx_wr;
} static blemsgs;

/* BLE communication structure */
struct
{
  uint8_t tx_buf[BG_DATA_LENGTH];
  uint8_t tx_wr;
  volatile uint8_t tx_rd;
  uint8_t rx_buf[BG_DATA_LENGTH];
  volatile uint8_t rx_wr;
  uint8_t rx_rd;
  volatile enum BgRxState rx_state;
  volatile bool req_exec;
  bool booted;
  bool connection;
  bool data_Connection;
  struct gecko_cmd_packet *evt;
} static ble;


/* BGM111 module low level init */

void BGM111_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the USART peripheral */
  RCC_APB1PeriphClockCmd(BGM111_UART_CLK, ENABLE);
  /* Enable TX, RX, CTS and RTS GPIO clocks */
  RCC_AHBPeriphClockCmd(BGM111_UART_TX_GPIO_CLK | BGM111_UART_RX_GPIO_CLK |
                        BGM111_UART_CTS_GPIO_CLK | BGM111_UART_RTS_GPIO_CLK |
                        BGM111_RESET_GPIO_CLK, ENABLE);
  
  /* Set TX, RX, CTS and RTS pin functions */
  GPIO_PinAFConfig(BGM111_UART_GPIO_PORT, BGM111_UART_TX_SOURCE, BGM111_UART_TX_AF);
  GPIO_PinAFConfig(BGM111_UART_GPIO_PORT, BGM111_UART_RX_SOURCE, BGM111_UART_RX_AF);
  GPIO_PinAFConfig(BGM111_UART_GPIO_PORT, BGM111_UART_CTS_SOURCE, BGM111_UART_CTS_AF);
  GPIO_PinAFConfig(BGM111_UART_GPIO_PORT, BGM111_UART_RTS_SOURCE, BGM111_UART_RTS_AF);
  
  /* USART TX, RX, CTS and RTS pin configuration */
  GPIO_InitStructure.GPIO_Pin = BGM111_UART_TX_PIN | BGM111_UART_RX_PIN |
                                BGM111_UART_CTS_PIN | BGM111_UART_RTS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(BGM111_UART_GPIO_PORT, &GPIO_InitStructure);
  
  /* BGM111 reset pin configuration */
  GPIO_InitStructure.GPIO_Pin = BGM111_RESET_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(BGM111_RESET_GPIO_PORT, &GPIO_InitStructure);
  
  /* USART configuration */
  USART_InitStructure.USART_BaudRate = BGM111_UART_BAUDRATE;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  
  /* Apply USART configuration */
  USART_Init(BGM111_UART, &USART_InitStructure);

  /* Turn on USART interrupt for RX and errors */
  USART_ITConfig(BGM111_UART, USART_IT_RXNE, ENABLE);
  USART_ITConfig(BGM111_UART, USART_IT_ERR, ENABLE);
  
  /* Enable the USART interrupt in the NVIC */
  NVIC_InitStructure.NVIC_IRQChannel = BGM111_UART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the BGM111 module USART */
  USART_Cmd(BGM111_UART, ENABLE);
  
  /* Housekeeping Init. */
  blemsgs.tx_wr = 0;
  ble.booted =  false;
  ble.connection = false;
  ble.data_Connection = false;
}

/* Next buffer index based on current index and buffer size */

//#pragma inline=forced
uint8_t NextBufIdx(uint8_t idx)
{
  idx++;
  return idx < BG_DATA_LENGTH ? idx : 0;
}

/* Report if the buffer is full based on its indexes */

//#pragma inline=forced
bool IsBufFull(uint8_t wr_idx, uint8_t rd_idx)
{
  return NextBufIdx(wr_idx) == rd_idx;
}

/* Get the used space in the buffer based on its indexes */

//#pragma inline=forced
uint8_t BufUsed(uint8_t wr_idx, uint8_t rd_idx)
{
  int size = (int)wr_idx - (int)rd_idx;
  if (size < 0)
  {
    size = BG_DATA_LENGTH + size;
  }
  return size;
}

/* Get the free space in the buffer based on its indexes */

//#pragma inline=forced
uint8_t BufFree(uint8_t wr_idx, uint8_t rd_idx)
{
  return (BG_DATA_LENGTH - 1) - BufUsed(wr_idx, rd_idx);
}

/* Receive data from the BGM111 module */

int BGM111_Receive(uint32_t len, uint8_t *data)
{
  
  /* Wait until we received the requested number of data bytes */
  while (len)
  {
    /* Is there a byte in the receive buffer? */
    if (ble.rx_rd != ble.rx_wr)
    {
      /* Disable interrupt */
      NVIC_DisableIRQ(BGM111_UART_IRQn);
      /* Clear the request to execute the BLE stack now we're doing it */
      ble.req_exec = false;
      /* Write to the output buffer */
      *data++ = ble.rx_buf[ble.rx_rd];
      /* Next byte in the receive buffer */
      ble.rx_rd = NextBufIdx(ble.rx_rd);
      /* Enable interrupt */
      NVIC_EnableIRQ(BGM111_UART_IRQn);

      /* One byte less to wait for */
      len--;
    }
    else
    {
      // Time to prime the pump again....
      /* Turn on USART interrupt for RX and errors */
      USART_ITConfig(BGM111_UART, USART_IT_RXNE, ENABLE);
      USART_ITConfig(BGM111_UART, USART_IT_ERR, ENABLE);
      break;
    }
  }
  /* No timeout, we always succeed since we already do packet parsing
   * in the interrupt handler */
  return 0;
}

/* Check whether there is input data from the BGM111 module */

int BGM111_Peek(void)
{
  return ble.req_exec;
}

/* Initialize the BGM111 module and BGLib */

void BGM111_Init(void)
{
  /* Perform low level init to initialize the UART for use with the 
   * BGM111 module */
  BGM111_LowLevel_Init();
  /* Pull the BGM111 reset pin low */
  GPIO_ResetBits(BGM111_RESET_GPIO_PORT, BGM111_RESET_PIN);
  /* Initialize BGLib with our transmit, receive and peek routines */
  BGLIB_INITIALIZE_NONBLOCK(BGM111_Transmit, BGM111_Receive, BGM111_Peek);
  // Wait 10 Msec before releasing Reset.
  delay_10ms();
  /* Release the BGM111 reset pin */
  GPIO_SetBits(BGM111_RESET_GPIO_PORT, BGM111_RESET_PIN);
}


/* Proces any input from the BLE module */
void BGM111_ProcessInput(void)
{
  /* Event structure */
  static uint32_t temp1;
  uint8_t tempBffr2[20];
  bool Boot_evt = false;

  /* Check whether there is an event to service */
  if (!ble.evt)
  {
    ble.evt = gecko_peek_event();
  }
  if (ble.evt)
  {
    // Log Event
    blemsgs.msgBuff[blemsgs.tx_wr] = BGLIB_MSG_ID(ble.evt->header);
    blemsgs.tx_wr = NextBufIdx(blemsgs.tx_wr);
    
    /* Service based on event header message ID */
    temp1 = BGLIB_MSG_ID(ble.evt->header);
    switch(temp1)
    //switch (BGLIB_MSG_ID(ble.evt->header))
    {
      /* System boot handler */
      case gecko_evt_system_boot_id:
        SkyPack_MNTR_UART_Transmit( (uint8_t *)"<<<BGM_BOOT>>>\r\n" );
        SendApp_String( (uint8_t *)"<<<BGM_BOOT>>>");
        /* Flag that the BLE module has booted */
        ble.booted = true;
        /* Fallthrough intentional */
      /* Connection closed handler */
      case gecko_evt_le_connection_closed_id:
        ble.connection = false;
        /* Set GAP mode: discoverable and connectable */
        if (gecko_cmd_le_gap_set_mode(le_gap_general_discoverable,
                    le_gap_undirected_connectable)->result < bg_errspc_bg)
        {
          // The following is a simple patch...Best way right now to recover is to force HARD Reset....
          if (Boot_evt == false)
          {
            SkyPack_MNTR_UART_Transmit( (uint8_t *)"<<<BGM_CNCTCLOSE>>>\r\n" );
            SendApp_String( (uint8_t *)"<<<BGM_CNCTCLOSE>>>");
            SkyPack_Reset( FATAL_CNCTDROP );
          }
          /* We succeeded, don't handle this event again */
          ble.evt = NULL;
        }
        break;
      //case 0x000800A0:
      case gecko_evt_le_connection_opened_id:
        /* Open Event...Set Active Connection Flag */
        /* Don't handle this event again */
        SkyPack_MNTR_UART_Transmit( (uint8_t *)"<<<BGM_CNCTOPEN>>>\r\n" );
        SendApp_String( (uint8_t *)"<<<BGM_CNCTOPEN>>>");
        ble.connection = true;
        ble.evt = NULL;
        break;
      // gecko_evt_gatt_server_user_write_request_id
      case 0x020A0020:
        // Clear Heart Beat... We have detected it.
        SkyPack_MNTR_UART_Transmit( (uint8_t *)"-*-" );
        Clr_HeartBeat();
        ble.evt = NULL;
        break;
      //case 0x030A0000:
      case gecko_evt_gatt_server_characteristic_status_id:
        // Clear Heart Beat... We have detected it.
        Flicker_Led();
        SkyPack_MNTR_UART_Transmit( (uint8_t *)"-O-" );
        Clr_HeartBeat();
        ble.evt = NULL;
        break;
      case 0x080000A0:
      case 0x000000A0:
      case 0x200000A0:
      case 0x050A0020:
//      case 0x200000A0:
//      case gecko_cmd_gatt_server_send_characteristic_notification_id: */
      /* Dummy catchall */
        /* Don't handle this event again */
        ble.evt = NULL;
        break;
      case gecko_rsp_gatt_read_characteristic_value_by_uuid_id:
      case 0x020800A0:
      case 0x000900A0:
        // Clear Heart Beat... We have detected it.
        SkyPack_MNTR_UART_Transmit( (uint8_t *)"<rsp_read_ch_value_by_uuid_id>");
        if (ble.evt->data.evt_gatt_server_attribute_value.value.len != 0)
        {
          sprintf( (char *)tempBffr2, "<STR:%s>", ble.evt->data.evt_gatt_server_attribute_value.value.data);
          SkyPack_MNTR_UART_Transmit( tempBffr2 );
        }
        ble.evt = NULL;
        break;
      case gecko_evt_endpoint_status_id:
       SkyPack_MNTR_UART_Transmit( (uint8_t *)"<evt_endpoint_status_id>");
        ble.evt = NULL;
        break;
      /* Dummy catchall */
      default:
        sprintf( (char *)tempBffr2, "<UNKN:%08x>", temp1);
        SkyPack_MNTR_UART_Transmit( tempBffr2);
        /* Don't handle this event again */
        ble.evt = NULL;
        break;
    };
    // Test RX Buffer and Set req_exec flag.
    if (BufUsed(ble.rx_wr, ble.rx_rd) != 0)
    {
      /* Indicate we need to execute the BLE stack to process 
       * the received packet */
      ble.req_exec = true;
    }
  } // EndIf (ble.evt)
}

/* BLE write characteristic */

void BGM111_WriteCharacteristic(uint8_t handle, uint8_t len, uint8_t *data)
{
  /* Write the attribute */
  gecko_cmd_gatt_server_write_attribute_value(handle, 0, len, data);
  /* Also trigger notify if enabled */
  gecko_cmd_gatt_server_send_characteristic_notification(0xFF,
            handle, len, data);
}

/**
  * @brief  Check whether the BLE module has booted and is ready for a command.
  * @retval bool:         true(1):        BGAPI IS ready.
  *                       false(0):         BGAPI NOT ready.
  */
bool BGM111_Ready(void)
{
  return ble.booted;
}

/**
  * @brief  Check whether the BLE module is connected.
  * @retval bool:         true(1):        Connection is Active.
  *                       false(0):       NO Connection.
  */
bool BGM111_Connected(void)
{
  return ble.connection;
}

/**
  * @brief  Check whether the BLE module is DATA connected.
  * @retval bool:         true(1):        DATA Connection is Active.
  *                       false(0):       NO DATA Connection.
  */
bool BGM111_DataConnected(void)
{
  return ble.data_Connection;
}

/**
  * @brief  This interrupt handler is called to handle the Usart3 interruptes.
  *         from the BGM111
  * @param  None
  * @retval None
  */
void BGM111_UART_IRQHandler(void)
{
//  static uint8_t header_cnt, payload_cnt, payload_len;
//  uint8_t tempBffr2[10];
  
  /* Transmit register empty? */
  if (USART_GetITStatus(BGM111_UART, USART_IT_TXE) == SET)
  {
    /* Are we still sending data? */
    if (ble.tx_rd != ble.tx_wr)
    {
      /* Send a byte */
      USART_SendData(BGM111_UART, ble.tx_buf[ble.tx_rd]);
      /* Bump the index */
      ble.tx_rd = NextBufIdx(ble.tx_rd);
    }
    else
    {
      /* Turn off the transmit interrupt */
      USART_ITConfig(BGM111_UART, USART_IT_TXE, DISABLE);
    }
  }
  
  /* Was there an error? */
  if (USART_GetITStatus(BGM111_UART, USART_IT_ORE_RX | 
      USART_IT_ORE_ER | USART_IT_NE | USART_IT_FE) == SET)
  {
    /* Reset the receive state */
    ble.rx_state = BGRX_SYNC;
    /* Clear the error by reading the data register */
    USART_ReceiveData(BGM111_UART);
    /* We're done */
    return;
  }

  /* Was a new byte received? */
  if (USART_GetITStatus(BGM111_UART, USART_IT_RXNE) == SET)
  {
    /* Get the byte (this also clears the flag) */
    uint8_t c = USART_ReceiveData(BGM111_UART);
    // TEST CODE....
    //sprintf( (char *)tempBffr2, "<%02x>", c);
    //SkyPack_MNTR_UART_Transmit(tempBffr2);
    //SkyPack_MNTR_UART_Transmit( (uint8_t *)"O" );
    /* Process the received character... */
    SkyBrd_ProcessBGMChar(c);
  }
}

/* Transmit data to the BGM111 module */

    
/**
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: Select the USART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval The received data.
  */
void BGM111_Transmit(uint32_t len, uint8_t *data)
{
  uint32_t tempmsg;
  uint32_t timeout_Cnt;
  
  // Log MSG into buffer
  tempmsg = 0xff000000 | len;
  blemsgs.msgBuff[blemsgs.tx_wr] = tempmsg;
  blemsgs.tx_wr = NextBufIdx(blemsgs.tx_wr);
  tempmsg = (uint32_t)(data[0]<<24) + (uint32_t)(data[1]<<16) + (uint32_t)(data[2]<<8) + (uint32_t)(data[3]);
  blemsgs.msgBuff[blemsgs.tx_wr] = tempmsg;
  blemsgs.tx_wr = NextBufIdx(blemsgs.tx_wr);
  tempmsg = (uint32_t)(data[4]<<24) + (uint32_t)(data[5]<<16) + (uint32_t)(data[6]<<8) + (uint32_t)(data[7]);
  blemsgs.msgBuff[blemsgs.tx_wr] = tempmsg;
  blemsgs.tx_wr = NextBufIdx(blemsgs.tx_wr);
  
  /* Add bytes when there is space, wait if necessary */
  // OK...But we Can't wait forever...Will place a timeout Count.
  timeout_Cnt = TX_TIMEOUT_CNT;
  
  while (len) {
    /* Get the next write index */
    uint8_t nextidx = NextBufIdx(ble.tx_wr);
    /* We're not hitting the read index? (There is space?) */
    if (nextidx != ble.tx_rd)
    {
      /* Put the data in the buffer */
      ble.tx_buf[ble.tx_wr] = *data;

      /* Disable interrupt */
      NVIC_DisableIRQ(BGM111_UART_IRQn);
      /* Trigger an interrupt if we're starting with an empty buffer */
      if (ble.tx_wr == ble.tx_rd)
      {
        USART_ITConfig(BGM111_UART, USART_IT_TXE, ENABLE);
      }
      /* Increment the write index */
      ble.tx_wr = nextidx;
      /* Enable interrupt */
      NVIC_EnableIRQ(BGM111_UART_IRQn);

      /* Update the incoming data length and pointer */
      len--;
      data++;
      //Found Space...Reset Count.
      timeout_Cnt = TX_TIMEOUT_CNT;
    }
    else
    {
      //Decrement Count
      if ( timeout_Cnt-- == 0)
      {
        // If this happens...NOT BOOTED..Clear ths flag or infinite loop.
        ble.booted = false;
        // We have detected a ERROR_TXBGMBUF_FULL error on BGM111...Log it!
        SkPck_ErrCdLogErrCd( ERROR_TXBGMBUF_FULL, MODULE_bgm111 );
        SkyPack_Reset( FATAL_TXBGMBUF_FULL );
      }
    }
  }
}

/**
  * @brief  This routine Process the passed character as part of the process of building status
  *         from the BGM111
  * @param  uint8_t c: Character to process.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef SkyBrd_ProcessBGMChar(uint8_t c)
{
//  static uint8_t header_cnt, payload_cnt, payload_len;
  HAL_StatusTypeDef Status;
  static uint8_t tempBffr2[40];
  uint8_t tempBffr3[60];
  static uint8_t in_ptr = 0;
  
  Status = HAL_OK;
  
  // First pull new character into buffer.
  tempBffr2[in_ptr++] = c;
  // Now, Did we get a termination character?
  if( c == 0x0a )
  {
    // Reset Ptr.
    in_ptr = 0;

    // Yes...We will now test contents of buffer. And then reset ptr back to 0.
    sprintf( (char *)tempBffr3, "\r\n<<FULL STRING>>: %s \r\n", tempBffr2);
    Status = SkyPack_MNTR_UART_Transmit( tempBffr3 );
    if (Status != HAL_OK)
      return Status;
    // Test Strings for Key items.
    // Boot String?
    if (strncmp((char *)tempBffr2,"Boot",4) == 0)
    {
      // Yes....Set Boot Flag.
      ble.booted = true;
      Status = SkyPack_MNTR_UART_Transmit((uint8_t *)"<ble.booted> ");

    }
    // Connection String?
    else if (strncmp((char *)tempBffr2,"Connected",9) == 0)
    {
      // Yes....Set Boot Flag.
      ble.connection = true;
      Status = SkyPack_MNTR_UART_Transmit((uint8_t *)"<ble.connection> ");

    }
    // Disconnection String?
    else if (strncmp((char *)tempBffr2,"Disconnected",12) == 0)
    {
      // Yes....Clear Flags.
      ble.connection = false;
      ble.data_Connection = false;
      ClrDataStructure();                           // Clear Backup data structure.
      ClrAnalyticsRepeat();                          // Clear Frame Repeat Count.
      Status = SkyPack_MNTR_UART_Transmit((uint8_t *)"<DISCONNECTED> ");
    }
    // Data String?
    else if (strncmp((char *)tempBffr2,"DATA",4) == 0)
    {
      // Yes....Set Boot Flag.
      ble.data_Connection = true;
      Status = SkyPack_MNTR_UART_Transmit((uint8_t *)"<ble.data_Connection> ");
    }
    // NOW TEST FOR PARAMS!!!

    // NOW TEST ERROR CONDITIONS!!!
    // OVERFLOW?
    else if (strncmp((char *)tempBffr2,"OVERFLOW",8) == 0)
    {
      // Yes....Report Error and Reset.
      // We have detected a ERROR_TXBGMBUF_FULL error on BGM111...Log it!
      SkPck_ErrCdLogErrCd(  ERROR_BGM_OVERFLOW, MODULE_bgm111 );
      SkyPack_Reset( ERROR_BGM_OVERFLOW );
    }
    else
    {
      Status = SkyPack_MNTR_UART_Transmit((uint8_t *)"<UNKNOWN STATUS> ");
    }
  }
  //tempBffr2[0] = c;
  //tempBffr2[1] = 0x00;
  //Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
  return Status;
#if 0
    Status = HAL_OK;
    // Test Buffer. If we are full, An error has occured. Must at least log that.
    if (IsBufFull(ble.rx_wr, ble.rx_rd))
    {
      RdBrd_ErrCdLogErrCd( ERROR_BGMBUF_FULL, MODULE_bgm111 );
      Clr_HrtBeat_Cnt();
      RdBrd_BlinkErrCd( ERROR_BGMBUF_FULL );
      //RoadBrd_Delay( 1000 );
      HAL_NVIC_SystemReset();
      return HAL_ERROR;
    }
    /* Execution based on state */
    switch (ble.rx_state)
    {
      /* Waiting for a valid start of header */
      default:
      case BGRX_SYNC:
        /* Valid start of header? (response or event) */
        if ((c & 0xF8) ==
            ((uint8_t)gecko_dev_type_gecko | (uint8_t)gecko_msg_type_rsp) ||
            (c & 0xF8) ==
            ((uint8_t)gecko_dev_type_gecko | (uint8_t)gecko_msg_type_evt))
        {
          /* Receiving header */
          ble.rx_state = BGRX_HDR;
          /* Initialize header byte counter */
          header_cnt = 0;
        }
        else
        {
          // Oops...Detected a fatal error...RESET!!!
          SkyPack_Reset( FATAL_SYNC );
          /* Stay in sync state until we receive a valid start of header */
          break;
        }
        /* Fallthrough intentional */
      /* Receiving header */
      case BGRX_HDR:
        if (IsBufFull(ble.rx_wr, ble.rx_rd))
        {
          // Oops...Detected a fatal error...RESET!!!
          SkyPack_Reset( FATAL_OVERFLOW );
          /* Indicate we need to execute the BLE stack, it's the
           * only way to get more space in the buffer */
          ble.req_exec = true;
          /* We're back to synchronizing */
          ble.rx_state = BGRX_SYNC;
        }
        else
        {
          /* Save the received byte */
          ble.rx_buf[ble.rx_wr] = c;
          /* Increment the index and header byte counter */
          ble.rx_wr = NextBufIdx(ble.rx_wr);
          header_cnt++;
          /* If this is the second header byte, we can grab the payload
           * length.  We ignore the first byte, since the spec says that
           * due to memory limitations in the modules, the packet is
           * never more than 64 bytes. */
          if (header_cnt == 2)
          {
            /* Get the payload length */
            payload_len = c;
            /* If we have a payload bigger than 60 bytes, something's wrong */
            if (payload_len > 60)
            {
              // Oops...Detected a fatal error...RESET!!!
              SkyPack_Reset( FATAL_PAYLDSYNC );
              /* Reset receive state to synchronizing */
              ble.rx_state = BGRX_SYNC;
              /* Indicate we need to execute the BLE stack to free space */
              ble.req_exec = true;
            }
          } // EndIf (header_cnt == 2)
          /* Are we done with the header? */
          if (header_cnt >= BGLIB_MSG_HEADER_LEN)
          {
            /* Is there no payload? */
            if (payload_len == 0)
            {
              /* Reset receive state to synchronizing */
              ble.rx_state = BGRX_SYNC;
              /* Indicate we need to execute the BLE stack to process 
               * the received packet */
              ble.req_exec = true;
            }
            else
            {
              /* Start receiving payload data */
              ble.rx_state = BGRX_DATA;
              /* Initialize the payload counter */
              payload_cnt = 0;
            }
          } // EndIf (header_cnt >= BGLIB_MSG_HEADER_LEN)
        } // EndElse (IsBufFull(ble.rx_wr, ble.rx_rd))
        break;
      /* Receiving data */
      case BGRX_DATA:
        /* Did we receive a byte, but the buffer is full? */
        if (IsBufFull(ble.rx_wr, ble.rx_rd))
        {
          // Oops...Detected a fatal error...RESET!!!
          SkyPack_Reset( FATAL_OVERFLOW );
          /* Indicate we need to execute the BLE stack, it's the
           * only way to get more space in the buffer */
          ble.req_exec = true;
          /* We're back to synchronizing */
          ble.rx_state = BGRX_SYNC;
        }
        else
        {
          /* Store the byte */
          ble.rx_buf[ble.rx_wr] = c;
          /* Increment the index and payload byte counter */
          ble.rx_wr = NextBufIdx(ble.rx_wr);
          payload_cnt++;
          /* Is this the end of the packet? */
          if (payload_cnt >= payload_len)
          {
            /* Reset receive state to synchronizing */
            ble.rx_state = BGRX_SYNC;
            /* Indicate we need to execute the BLE stack to process 
             * the received packet */
            ble.req_exec = true;
          }
          /* Is the buffer almost full? */
          if (BufFree(ble.rx_wr, ble.rx_rd) <= BG_DATA_LOW_WATERMARK)
          {
            /* Indicate we need to execute the BLE stack so it can
             * start reading data from the buffer */
            ble.req_exec = true;
          }
        } // EndElse (IsBufFull(ble.rx_wr, ble.rx_rd))
        break;
    }
  }
#endif
}

/**
  * @brief  This routine returns the status of the ble.req_exec flag
  *         from the BGM111
  * @param  None
  * @retval bool:     true(1):     Processing of packet about to start
  *                   false(0):      No processing pending.
  */
bool RoadBrd_tstReqexec( void )
{
  return ble.req_exec;
}


