/*
*  @file    target.c
*  @brief   Method1 - TOA: Source Code for Target
*  @author 	Stefan Koller, BSc
*/

#include "target.h"

static uint8 tx_msg[UWB_MSG_LENGTH];
static uint8 eui_target[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0xCA, 0xDE};
static uint8 rx_uwb_msg[UWB_MSG_LENGTH];
static uint8 buffer_uart[UART_BUFF_LENGTH];
//static uint8 eui[8];

void fill_fixed_txMsg(void)
{
  // Structure according to msg_id_toa from ATLAS

  // EUI from tx-Node (SyncNode)
  memcpy(&tx_msg[0], &eui_target[0], 8);
  
  // The rest needs to be filled dynamically
}

void wait_for_GO(void)
{
  uint32_t err_code = 0;
  uint8 i = 0;

  printf("Received buffer_uart = ");
  for(i = 0; i < WAIT_FOR_GO_LENGTH; i++)
  {
      err_code = app_uart_get(&buffer_uart[i]);
      if(err_code == NRF_SUCCESS)
      {
        printf("%c ", buffer_uart[i]);
      }
      else if(err_code == NRF_ERROR_NOT_FOUND)
      {
        // Nothing received yet. 
        // Decrement i so that first buffer element is referenced in next call again.
        i--;
      }
      else
      {
        printf("UART_get ERROR \r\n");
        printf("Error number UART_get = %d \r\n", err_code);
      }   
  }
  printf("\r\n");
  if(buffer_uart[0] == 'G')
  {
    if(buffer_uart[1] == 'O')
    {
      printf("Target: Continous functionality started. \r\n");
    }
    else
    {
      printf("ERROR: Wrong command for startin continous functionality. Restart required.\r\n");
      assert(FALSE);
    }
  }
  else
  {
    printf("ERROR: Wrong command for startin continous functionality. Restart required.\r\n");
    assert(FALSE);
  }
}

void target_task_function (void * pvParameter)
{
  uint64_t default_txts = 0xFFFFFFFFFFFFFFFF;
  uint8_t txstamp[8] = {0x00};
  static uint8_t counter = 0; 
  static uint8 frame_seq_nb = 1;
  uint8_t rx_ts[8] = {0};
  uint32_t rx_ts_hi = 0;
  uint32_t rx_ts_lo = 0;
  uint64_t resp_tx_time = 0;
  uint64_t resp_tx_ts = 0;
  uint64_t systime_u64 = 0;
  int i = 0;
  uint32_t err_code = 0;
  static uint32 status_reg = 0;
  UNUSED_PARAMETER(pvParameter);
  while (true)
  {
    //vTaskDelay(300);

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    // Receive UWB (blocking)
    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 4 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {};

    #if 0  // include if required to help debug timeouts.
    int temp = 0;		
    if(status_reg & SYS_STATUS_RXFCG )
    temp =1;
    else if(status_reg & SYS_STATUS_ALL_RX_TO )
    temp =2;
    if(status_reg & SYS_STATUS_ALL_RX_ERR )
    temp =3;
    #endif

    if (status_reg & SYS_STATUS_RXFCG)
    {		
      uint32 frame_len;

      /* Clear good RX frame event in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

      /* A frame has been received, read it into the local buffer. */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
   
      if (frame_len <= UWB_MSG_LENGTH)
      {
        dwt_readrxdata(rx_uwb_msg, frame_len, 0);
      }
      dwt_readrxtimestamp(rx_ts);
      rx_ts_lo = dwt_readrxtimestamplo32();
      rx_ts_hi = dwt_readrxtimestamphi32();

      /* enhanced for understanding */
      printf("Rx timestamp low  = %x\r\n",rx_ts_lo);
      printf("Rx timestamp high = %x\r\n",rx_ts_hi);

      // Fill UART buffer
      // Structure according to msg_id_toa from ATLAS
      memcpy(&buffer_uart[0], &rx_uwb_msg[0], 8);
      memcpy(&buffer_uart[8], &eui_target[0], 8);
      memcpy(&buffer_uart[16], &rx_ts[0], 8);
      /*
      buffer_uart[16] =  rx_ts_lo;
      buffer_uart[17] = (rx_ts_lo >> 8);
      buffer_uart[18] = (rx_ts_lo >> 16);
      buffer_uart[19] = (rx_ts_lo >> 24);  
      buffer_uart[20] =  rx_ts_hi;
      buffer_uart[21] = (rx_ts_hi >> 8);
      buffer_uart[22] = (rx_ts_hi >> 16);
      buffer_uart[23] = (rx_ts_hi >> 24);  
      */
      memcpy(&buffer_uart[24], &rx_uwb_msg[24], 8);
      memcpy(&buffer_uart[32], &rx_uwb_msg[32], 1);

      // send UART buffer
      printf("Sent UART Buffer = ");
      for(i = 0; i < UART_BUFF_LENGTH; i++)
      {
          err_code = app_uart_put(buffer_uart[i]);
          if(err_code == NRF_SUCCESS)
          {
            printf("%x ", buffer_uart[i]);
          }
          else
          {
            printf("UART_send ERROR \r\n");
            printf("Error number UART_send = %d \r\n", err_code);
          }   
      }  
      printf("\r\n");

      // Toggle the LED for indication
      LEDS_INVERT(BSP_LED_3_MASK);
      counter++;

      // if 10 times executed, 1 second is over and the target should send its "ping"
      if(counter == 8)
      //if(counter == 18)
      {
        LEDS_INVERT(BSP_LED_0_MASK);

        // Write frame data to DW1000 and prepare first transmission
        tx_msg[POS_SEQNUM] = frame_seq_nb;
        printf("SeqNum first transmission = %d\r\n", frame_seq_nb);

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        // Fill the default time stamp to the transmit buffer
        memcpy(&tx_msg[24], &default_txts, 8);

        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); // Zero offset in TX buffer.
        dwt_writetxfctrl(sizeof(tx_msg), 0, 1); // Zero offset in TX buffer, ranging.

        // Start transmission, indicating that no response is expected
        dwt_starttx(DWT_START_TX_IMMEDIATE);
/*
        counter = 0;
        LEDS_INVERT(BSP_LED_0_MASK);
        uint8 systime[8] = {0};

        // Write frame data to DW1000 and prepare transmission
        tx_msg[POS_SEQNUM] = frame_seq_nb;
        printf("SeqNum = %d\r\n", frame_seq_nb);

        if(frame_seq_nb < 0xFF)
        {
          frame_seq_nb++;
        }
        else
        {
          // Reinizialize the frame sequence number if uint8 is overflowed
          frame_seq_nb = 0; 
        }

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);


        // Fill the current time stamp to the transmit buffer
        uint32_t systime_high = dwt_readsystimestamphi32();
        //printf("systime_high = %d\r\n", systime_high);
        //systime_u64 = (systime_high + 68000); 
        // Sift left for 1 Byte, as the last byte will always be zero according API guide
        //systime_u64 = (systime_u64 << 8);
        //memcpy(&tx_msg[24], &systime_u64, 8);
        
        // Compute final message transmission time. See NOTE 7 below.
        resp_tx_time = (systime_u64 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME));
        dwt_setdelayedtrxtime(resp_tx_time);

        // Response TX timestamp is the transmission time we programmed plus the antenna delay.
        resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        //printf("resp_tx_ts = %d\r\n", resp_tx_ts);
        memcpy(&tx_msg[24], &resp_tx_ts, 8);

        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); // Zero offset in TX buffer.
        dwt_writetxfctrl(sizeof(tx_msg), 0, 1); // Zero offset in TX buffer, ranging.

        //uint64_t systime_before_send = dwt_readsystimestamphi32();
        //systime_before_send = (systime_before_send << 8);
        //printf("systime before send = %d\r\n", dwt_readsystimestamphi32());
        // Start transmission, indicating that no response is expected
        //dwt_starttx(DWT_START_TX_IMMEDIATE);
        dwt_starttx(DWT_START_TX_DELAYED);

        // Poll DW1000 until TX frame sent event set. See NOTE 5 below
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        {};

        uint64_t systime_after_send = dwt_readsystimestamphi32();
        dwt_readtxtimestamp(systime_txstamp);
        systime_after_send = (systime_after_send << 8);

        // Clear TXFRS event
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        printf("---------------------------------------");
*/

      }
      else if(counter >= 10)
      //else if(counter >= 20)
      {
        counter = 0;
        LEDS_INVERT(BSP_LED_0_MASK);

        // Write frame data to DW1000 and prepare second transmission
        tx_msg[POS_SEQNUM] = frame_seq_nb;
        printf("SeqNum second transmission = %d\r\n", frame_seq_nb);
        
        if(frame_seq_nb < 0xFF)
        {
          frame_seq_nb++;
        }
        else
        {
          // Reinizialize the frame sequence number if uint8 is overflowed
          frame_seq_nb = 1; 
        }

        dwt_readtxtimestamp(txstamp);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        // Fill the default time stamp to the transmit buffer
        memcpy(&tx_msg[24], &txstamp[0], 8);

        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); // Zero offset in TX buffer.
        dwt_writetxfctrl(sizeof(tx_msg), 0, 1); // Zero offset in TX buffer, ranging.

        // Start transmission, indicating that no response is expected
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        printf("--------------------------\r\n");
      }
      else
      {
        //Do nothing, can not happen.
      }
    }
    else
    {
      /* Clear RX error/timeout events in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

      /* Reset RX to properly reinitialise LDE operation. */
      dwt_rxreset();
    }
  }
}

 /*
void target_uwb_tx_callbackfkt (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  LEDS_INVERT(BSP_LED_0_MASK);
  uint8 systime[8] = {0};

  // Write frame data to DW1000 and prepare transmission.
  //poll_tx_ts = dwt_readtxtimestamplo32();
  tx_msg[POS_SEQNUM] = frame_seq_nb;
  printf("SeqNum = %d\r\n", frame_seq_nb);
  
  if(frame_seq_nb < 0xFF)
  {
    frame_seq_nb++;
  }
  else
  {
    // Reinizialize the frame sequence number if uint8 is overflowed
    frame_seq_nb = 0; 
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

  // Fill the current time stamp to the transmit buffer
  dwt_readsystime(systime);
  memcpy(&tx_msg[23], &systime[0], 8);

  dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); // Zero offset in TX buffer.
  dwt_writetxfctrl(sizeof(tx_msg), 0, 1); // Zero offset in TX buffer, ranging.

  // Start transmission, indicating that no response is expected
  dwt_starttx(DWT_START_TX_IMMEDIATE);
}
*/
