/*
*  @file    anchor.h
*  @brief   Method2 - TWR: Source Code for Anchor Nodes
*  @author 	Stefan Koller, BSc
*/

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "assert.h"

// #defines for general definitions
#define ANCHOR_NUMBER     0x04
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME   65536

// #defines for delays
#define RNG_DELAY_MS    50
// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

// #defines for buffer and message length definitions
#define TX_BUF_LEN_UWB    32 // 30 Byte data + 2 Byte checksum
#define RX_BUF_LEN_UWB    22 // 20 Byte data + 2 Byte checksum
#define BUF_LEN_UART      76
#define LEN_RESPMSG_TS     4
//#define WAIT_FOR_GO_LENGTH  2

// #defines for buffer positions
#define POS_SEQNUM           8
#define POS_EUITARGET        0
#define POS_EUIANCHORX      12
#define POS_RESPMSG_ARXTS   20
#define POS_RESPMSG_ATXTS   24
#define POS_DISTANCE        20

//extern void anchor_task_function (void * pvParameter);
int anchor_callback_function(void);
void fill_fixed_txMsg(void);
uint64 get_rx_timestamp_u64(void);
void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);