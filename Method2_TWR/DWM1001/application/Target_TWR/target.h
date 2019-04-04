/*
*  @file    target.h
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
#include "app_uart.h"
#include "assert.h"

// #defines for general definitions
#define MAX_ANCHORS   4
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME   65536
/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT  299702547

// #defines for delays
#define RNG_DELAY_MS    500

// #defines for buffer and message length definitions
#define TX_BUF_LEN_UWB    22 // 20 Byte data + 2 Byte checksum
#define RX_BUF_LEN_UWB    32 // 30 Byte data + 2 Byte checksum
#define BUF_LEN_UART      (16 + (MAX_ANCHORS * 16))
#define LEN_RESPMSG_TS     4
#define LEN_OFFSET_UWBTOUART 4

// #defines for buffer positions
#define POS_MSGID            0
#define POS_MSGLEN           2
#define POS_SEQNUM           8
#define POS_EUITARGET        0
#define POS_EUIANCHORX      12
#define POS_RESPMSG_ARXTS   20
#define POS_RESPMSG_ATXTS   24
#define POS_DISTANCE        (44 + LEN_OFFSET_UWBTOUART)

void fill_fixed_txMsg(void);
//void target_task_function (void * pvParameter);
int target_callbackfkt (void);
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);