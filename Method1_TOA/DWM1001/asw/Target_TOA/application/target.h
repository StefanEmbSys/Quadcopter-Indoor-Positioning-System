/*
*  @file    target.h
*  @brief   Method1 - TOA: Source Code for Target
*  @author 	Stefan Koller, BSc
*/

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "app_uart.h"
#include "assert.h"

#define WAIT_FOR_GO_LENGTH  2
#define UART_BUFF_LENGTH   33
#define UWB_MSG_LENGTH     35
#define POS_SEQNUM         32

// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Default antenna delay values for 64 MHz PRF.*/
#define TX_ANT_DLY 16456

void fill_fixed_txMsg(void);
void wait_for_GO(void);
void target_task_function (void * pvParameter);
extern void target_uwb_tx_callbackfkt (void * pvParameter);