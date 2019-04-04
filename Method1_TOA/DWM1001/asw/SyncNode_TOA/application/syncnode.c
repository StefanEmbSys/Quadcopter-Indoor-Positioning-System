/*
*  @file    syncnode.c
*  @brief   Method1 - TOA: Source Code for Sync Node
*  @author 	Stefan Koller, BSc
*/

#include "syncnode.h"

static uint8 tx_sync_msg[UWB_SYNCMSG_LENGTH] = {0};
static uint8 buffer_uart[UART_SYNCINI_LENGTH];

#ifdef USE_FREERTOS
 
void SyncPeriod_timer_callback (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  static uint8 frame_seq_nb = 1;
  uint64_t default_txts = 0xFFFFFFFFFFFFFFFF;
/*
  // get the current Tick of this task to know how accurate the task is executed
  static uint16_t x = 0;
  static TickType_t tick_now[1000];
  static uint64_t systime_u64[1000];
  static double systime_sec[1000];
  static double systime_sec_diff[1000];
  tick_now[x] = xTaskGetTickCount();
  systime_u64[x] = dwt_readsystimestamphi32();
  // Sift left for 1 Byte, as the last byte will always be zero according API guide
  systime_u64[x] = (systime_u64[x] << 8);
  systime_sec[x] = (double)(systime_u64[x] * DWT_TIME_UNITS);
  if(x > 1) systime_sec_diff[x-1] = (systime_sec[x] - systime_sec[x-1]);
  x++;
*/
  
  LEDS_INVERT(BSP_LED_1_MASK);

  // Write frame data to DW1000 and prepare transmission.
  tx_sync_msg[POS_SEQNUM] = frame_seq_nb;
  printf("SeqNum = %d\r\n", frame_seq_nb);
  
  if(frame_seq_nb < 0xFF)
  {
    frame_seq_nb++;
  }
  else
  {
    // Reinizialize the frame sequence number if uint8 is overflowed
    frame_seq_nb = 1; 
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  dwt_writetxdata(sizeof(tx_sync_msg), tx_sync_msg, 0); // Zero offset in TX buffer.
  dwt_writetxfctrl(sizeof(tx_sync_msg), 0, 1); // Zero offset in TX buffer, ranging.

  // Start transmission, indicating that no response is expected
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  printf("--------------------------\r\n");  
}
#else
  extern int ss_init_run(void);
#endif   // #ifdef USE_FREERTOS

void ini_SyncNode (void)
{
  uint32_t err_code = 0;
  uint8 x = 0;

  for(x = 0; x < UART_SYNCINI_LENGTH; x++)
  {
      err_code = app_uart_get(&buffer_uart[x]);
      if(err_code == NRF_SUCCESS)
      {
        printf("buffer_uart[%d] = %x \r\n", x, buffer_uart[x]);
      }
      else if(err_code == NRF_ERROR_NOT_FOUND)
      {
        // Nothing received yet. 
        // Decrement x so that first buffer element is referenced in next call again.
        x--;
      }
      else
      {
        printf("UART_get ERROR \r\n");
        printf("Error number UART_get = %d \r\n", err_code);
      }   
  }  
}

void fill_fixed_txSync(void)
{
  // Structure according to msg_id_toa from ATLAS

  // EUI from tx-Node (SyncNode)
  memcpy(&tx_sync_msg[0], &buffer_uart[5], 8);
  
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

