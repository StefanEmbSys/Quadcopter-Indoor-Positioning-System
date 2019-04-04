/*
*  @file    syncnode.h
*  @brief   Method1 - TOA: Source Code for Sync Node with Clock Correction extension
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
#include "app_uart.h"
#include "assert.h"
#include "dw1000/dw1000_pub.h"

#define UART_SYNCINI_LENGTH    13
#define WAIT_FOR_GO_LENGTH      2
#define UWB_SYNCMSG_LENGTH     35
#define POS_SEQNUM             32

extern void SyncPeriod_timer_callback (void * pvParameter);
void ini_SyncNode (void);
void fill_fixed_txSync(void);
void wait_for_GO(void);