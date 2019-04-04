/*
*  @file    anchor.c
*  @brief   Method2 - TWR: Source Code for Anchor Nodes
*  @author 	Stefan Koller, BSc
*/


#include "sdk_config.h" 
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "anchor.h"


/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

// declaration of static buffers and variables
static uint8 eui_target[] = {0xDE, 0xCA, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
static uint8 eui_anchorX[] = {0xDE, 0xCA, 0x02, 0x00, 0x00, 0x00, 0x00, ANCHOR_NUMBER};
//static uint8 buffer_uart[BUF_LEN_UART];
static uint8 rx_initiator_msg[RX_BUF_LEN_UWB];
static uint8 responder_msg[TX_BUF_LEN_UWB];
static uint32 status_reg = 0;


int anchor_callback_function(void)
{
  static uint32 frame_seq_nb = 1;
  static uint32 last_frame_seq_nb = 0;
  int ret = 0;

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
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
        printf("temp = %d\r\n",temp);
        #endif

    if (status_reg & SYS_STATUS_RXFCG)
    {
      uint32 frame_len;
      uint32 temp_frame_seq_nb = 0;

      /* Clear good RX frame event in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

      /* A frame has been received, read it into the local buffer. */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
      if (frame_len <= RX_BUF_LEN_UWB)
      {
        dwt_readrxdata(rx_initiator_msg, frame_len, 0);
      }
    
      // Allign received Seqence Number from uint8 buffer to a uint32 variable for comparison
      temp_frame_seq_nb = (rx_initiator_msg[POS_SEQNUM] << 24);
      temp_frame_seq_nb += (rx_initiator_msg[POS_SEQNUM + 1] << 16);
      temp_frame_seq_nb += (rx_initiator_msg[POS_SEQNUM + 2] << 8);
      temp_frame_seq_nb += (rx_initiator_msg[POS_SEQNUM + 3]);

      // Check that the frame is the expected
      // Check that the SeqNum from the Initiator message equals the last one +1
      // And Check that the EUI-Target and the EUI-AnchorX are correct
      if (memcmp(&rx_initiator_msg[POS_EUIANCHORX], &eui_anchorX, 8) == 0 )  /* EUI-AnchorX */
      {
        if (memcmp(&rx_initiator_msg[POS_EUITARGET], &eui_target, 8) == 0 )  /* EUI-Target */
        {
          if (temp_frame_seq_nb == (last_frame_seq_nb + 1))  /* SeqNum */
          {
            uint32 resp_tx_time;

            // Turn on LED D11 to show that a good Frame has been received and was accepted
            LEDS_ON(BSP_LED_2_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   
        
            /* Retrieve poll reception timestamp. */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Compute final message transmission time. See NOTE 7 below. */
            resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
            resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            // Write Sequence Number in tx message
            responder_msg[POS_SEQNUM] = (uint8) (frame_seq_nb >> 24);
            responder_msg[POS_SEQNUM + 1] = (uint8) (frame_seq_nb >> 16);
            responder_msg[POS_SEQNUM + 2] = (uint8) (frame_seq_nb >> 8);
            responder_msg[POS_SEQNUM + 3] = (uint8) (frame_seq_nb);
            // Write all timestamps in tx message
            resp_msg_set_ts(&responder_msg[POS_RESPMSG_ARXTS], poll_rx_ts);
            resp_msg_set_ts(&responder_msg[POS_RESPMSG_ATXTS], resp_tx_ts);

            // Write and send the response message      
            dwt_writetxdata(sizeof(responder_msg), responder_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
            dwt_writetxfctrl(sizeof(responder_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
            ret = dwt_starttx(DWT_START_TX_DELAYED);

            /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
            if (ret == DWT_SUCCESS)
            {
              /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
              while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
              {};

              /* Clear TXFRS event. */
              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
            }
            else
            {
              /* If we end up in here then we have not succeded in transmitting the packet we sent up.
              POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different processors. 
              For slower platforms where the SPI is at a slower speed or the processor is operating at a lower 
              frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz)this value needs to be increased.
              Knowing the exact time when the responder is going to send its response is vital for time of flight 
              calculation. The specification of the time of respnse must allow the processor enough time to do its 
              calculations and put the packet in the Tx buffer. So more time is required for a slower system(processor).
              */

              /* Reset RX to properly reinitialise LDE operation. */
              dwt_rxreset();
            }
          }
          // Increment frame sequence number
          last_frame_seq_nb = frame_seq_nb;
          frame_seq_nb++;
        }
      }
    }
    else
    {
      /* Clear RX error events in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

      /* Reset RX to properly reinitialise LDE operation. */
      dwt_rxreset();
    }
  return(1);		
}

void anchor_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setleds(DWT_LEDS_ENABLE);
  // Turn on LED D9 to show that init is complete and task function is executing
  LEDS_ON(BSP_LED_0_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   
  fill_fixed_txMsg();

  while (true)
  {
    anchor_callback_function();
    /* Delay a task for a given number of ticks */
    vTaskDelay(RNG_DELAY_MS);
    // Turn off LED D11 again to be prepared for the next accepted Frame
    LEDS_OFF(BSP_LED_2_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   
    /* Tasks must be implemented to never return... */
  }
}

void fill_fixed_txMsg(void)
{
  // EUI from target
  memcpy(&responder_msg[POS_EUITARGET], &eui_target[0], 8);

  // EUI from this anchor
  memcpy(&responder_msg[POS_EUIANCHORX], &eui_anchorX[0], 8);
  
  // The rest needs to be filled dynamically
}

uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < LEN_RESPMSG_TS; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}
