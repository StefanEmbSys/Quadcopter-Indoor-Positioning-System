/*
*  @file    basestation.h
*  @brief   Method1 - TOA: Source Code for Anchor (Base Station)
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

#define UART_INI_LENGTH     9
#define WAIT_FOR_GO_LENGTH  2
#define UART_BUFF_LENGTH   33
#define UWB_MSG_LENGTH     35
#define POS_SEQNUM         32

extern void basestation_task_function (void * pvParameter);
void ini_BaseStation (void);
void fill_UARTtxdata(void);
void wait_for_GO(void);