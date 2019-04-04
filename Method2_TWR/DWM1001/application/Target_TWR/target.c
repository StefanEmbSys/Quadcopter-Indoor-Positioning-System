/*
*  @file    target.c
*  @brief   Method2 - TWR: Source Code for Target
*  @author 	Stefan Koller, BSc
*/

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "target.h"


// declaration of static buffers and variables
static uint8 eui_target[] = {0xDE, 0xCA, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
static uint8 eui_anchorX[] = {0xDE, 0xCA, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFF};
static uint8 buffer_uart[BUF_LEN_UART];
static uint8 initiator_msg[TX_BUF_LEN_UWB];
static uint8 rx_responder_msg[RX_BUF_LEN_UWB];
static uint32 status_reg = 0;
static double tof[MAX_ANCHORS];
static double distance[MAX_ANCHORS];
static uint8 anchorX[MAX_ANCHORS][8];
static uint32 frame_seq_nb = 1;


void fill_fixed_txMsg(void)
{
  // EUI from target
  memcpy(&initiator_msg[POS_EUITARGET], &eui_target[0], 8);

  // EUI from anchor X (the last number is filled dynamically)
  memcpy(&initiator_msg[POS_EUIANCHORX], &eui_anchorX[0], 8);
  
  // The rest needs to be filled dynamically
}

int target_callbackfkt(void)
{
  uint32_t err_code = 0;
  uint8 anchor_cntr = 1;
  uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
  int32 rtd_init, rtd_resp;
  float clockOffsetRatio;
  int i, j;
    // Conducting TWR for distance measurement cyclically for every anchor node
    for(anchor_cntr = 1; anchor_cntr <= MAX_ANCHORS; anchor_cntr++)
    {
      // Turn on LED D12 to indicate TWR exchange
      LEDS_ON(BSP_LED_3_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   
      
      /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
      initiator_msg[POS_SEQNUM] = (uint8) (frame_seq_nb >> 24);
      initiator_msg[POS_SEQNUM + 1] = (uint8) (frame_seq_nb >> 16);
      initiator_msg[POS_SEQNUM + 2] = (uint8) (frame_seq_nb >> 8);
      initiator_msg[POS_SEQNUM + 3] = (uint8) (frame_seq_nb);
      initiator_msg[POS_EUIANCHORX + 7] = anchor_cntr; // +7 to reach the last position of the EUI-AnchorX

      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
      dwt_writetxdata(sizeof(initiator_msg), initiator_msg, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(sizeof(initiator_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

      /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
      * set by dwt_setrxaftertxdelay() has elapsed. */

      dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

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
        printf("temp = %d\r\n",temp);
        #endif
      
      if (status_reg & SYS_STATUS_RXFCG)
      {		
        uint32 frame_len;

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
   
        if (frame_len <= RX_BUF_LEN_UWB)
        {
          dwt_readrxdata(rx_responder_msg, frame_len, 0);
        }

        // Check that the frame is the expected response
        // Check that the SeqNum from the Initiator message equals the one responded
        // And Check that the EUI-Target is correct and EUI-AnchorX is matching with the one in the Initiator message
        if (memcmp(&rx_responder_msg[POS_SEQNUM], &initiator_msg[POS_SEQNUM], 4) == 0) /* SeqNum */
        {
          if (memcmp(&rx_responder_msg[POS_EUITARGET], &initiator_msg[POS_EUITARGET], 8) == 0 )  /* EUI-Target */
          {
            if (memcmp(&rx_responder_msg[POS_EUIANCHORX], &initiator_msg[POS_EUIANCHORX], 8) == 0 )  /* EUI-AnchorX */
            {	
              // Turn on LED D11 to show that a good Frame has been received and was accepted
              LEDS_ON(BSP_LED_2_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   

              /* Retrieve poll transmission and response reception timestamps. See NOTE 5 below. */
              poll_tx_ts = dwt_readtxtimestamplo32();
              resp_rx_ts = dwt_readrxtimestamplo32();

              /* enhanced for understanding */
              //printf("Tx timestamp initiator = %d\r\n", poll_tx_ts);
              //printf("Rx timestamp initiator = %d\r\n", resp_rx_ts);

              /* Read carrier integrator value and calculate clock offset ratio. See NOTE 7 below. */
              clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

              /* Get timestamps embedded in response message. */
              resp_msg_get_ts(&rx_responder_msg[POS_RESPMSG_ARXTS], &poll_rx_ts);
              resp_msg_get_ts(&rx_responder_msg[POS_RESPMSG_ATXTS], &resp_tx_ts);

              /* enhanced for understanding */
              //printf("Tx timestamp receiver = %d\r\n", resp_tx_ts);
              //printf("Rx timestamp receiver = %d\r\n", poll_rx_ts);

              /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
              rtd_init = resp_rx_ts - poll_tx_ts;
              rtd_resp = resp_tx_ts - poll_rx_ts;

              /* enhanced for understanding */
              //printf("time initiator to receiver = %d\r\n", rtd_init);
              //printf("time receiver to initiator = %d\r\n", rtd_resp);

              tof[anchor_cntr - 1] = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 

              /* enhanced for understanding */
              //printf("Time Of Flight = %d\r\n", tof[anchor_cntr - 1]);

              // Calculate the distance
              distance[anchor_cntr - 1] = tof[anchor_cntr - 1] * SPEED_OF_LIGHT;

              // Store the rx-EUI
              memcpy(&anchorX[anchor_cntr - 1][0], &rx_responder_msg[POS_EUIANCHORX], 8);

              // Print the distance
              printf("Distance of EUI %d : %f\r\n",anchor_cntr, distance[anchor_cntr - 1]);
              vTaskDelay(200);
            }
            else
            {
              //printf("ERROR EUI %d : Different Anchor EUI\r\n",anchor_cntr);
              /* Clear RX error/timeout events in the DW1000 status register. */
              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
              dwt_rxreset();
            }
          }
          else
          {
            printf("ERROR EUI %d : Different Target EUI\r\n",anchor_cntr);
            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
          }
        }
        else
        {
          printf("ERROR EUI %d : Different Sequence Number\r\n",anchor_cntr);
          /* Clear RX error/timeout events in the DW1000 status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
          dwt_rxreset();
        }
      }
      // Turn off LED D11 to be prepared for the next received Frame
      LEDS_OFF(BSP_LED_2_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   
    }
    frame_seq_nb++;
    // Turn off LED D12 to indicate TWR exchange finished to all anchors
    LEDS_OFF(BSP_LED_3_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   

    // Fill UART buffer for transmission of the distances to RPI
    buffer_uart[POS_MSGID] = 0x01;
    buffer_uart[POS_MSGID + 1] = 0x05;
    buffer_uart[POS_MSGLEN] = 0x00;
    buffer_uart[POS_MSGLEN + 1] = (BUF_LEN_UART - LEN_OFFSET_UWBTOUART);
    memcpy(&buffer_uart[POS_EUITARGET + LEN_OFFSET_UWBTOUART], &rx_responder_msg[POS_EUITARGET], 8);
    memcpy(&buffer_uart[POS_SEQNUM + LEN_OFFSET_UWBTOUART], &rx_responder_msg[POS_SEQNUM], 4);
    for(i = 0, j = 0; j < MAX_ANCHORS; (i = i + 8), j++)
    {
      memcpy(&buffer_uart[POS_EUIANCHORX + i + LEN_OFFSET_UWBTOUART], &anchorX[j], 8);
      memcpy(&buffer_uart[POS_DISTANCE + i], &distance[j], 8);
    }
  
    // send UART buffer to RPI
    printf("Sent UART Buffer = ");
    for(i = 0; i < BUF_LEN_UART; i++)
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
    return(1);
}

static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < LEN_RESPMSG_TS; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}

void target_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  dwt_setleds(DWT_LEDS_ENABLE);

  // Delay the task about 20 seconds to give the user time to reset the anchors.
  int i = 0;
  for(i = 0; i < 25; i++)
  {
    vTaskDelay(1000);
    LEDS_INVERT(BSP_LED_1_MASK);
  }
  LEDS_OFF(BSP_LED_1_MASK);

  // Turn on LED D9 to show that init is complete and task function is executing
  LEDS_ON(BSP_LED_0_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   
  fill_fixed_txMsg();
  while (true)
  {
    target_callbackfkt();
    /* Delay a task for a given number of ticks */
    vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */
  }
}
