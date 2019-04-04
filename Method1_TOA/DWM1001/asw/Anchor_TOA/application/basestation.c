/*
*  @file    basestation.c
*  @brief   Method1 - TOA: Source Code for Anchor (Base Station)
*  @author 	Stefan Koller, BSc
*/

#include "basestation.h"

static uint8 rx_uwb_msg[UWB_MSG_LENGTH];
static uint8 buffer_uart[UART_BUFF_LENGTH];
static uint8 eui[8];

void ini_BaseStation (void)
{
  uint32_t err_code = 0;
  uint8 i = 0;

  printf("Received Init buffer_uart = ");
  for(i = 0; i < UART_INI_LENGTH; i++)
  {
      err_code = app_uart_get(&buffer_uart[i]);
      if(err_code == NRF_SUCCESS)
      {
        printf("%x ", buffer_uart[i]);
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
  memcpy(&eui[0], &buffer_uart[1], 8);
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
      printf("BaseStation: Continous functionality started. \r\n");
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

void basestation_task_function (void * pvParameter)
{
  uint32_t rx_ts_hi = 0; 
  uint32_t rx_ts_lo = 0;
  uint8 rx_ts[8] = {0};
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
      memcpy(&buffer_uart[8], &eui[0], 8);
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
      LEDS_INVERT(BSP_LED_0_MASK);
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