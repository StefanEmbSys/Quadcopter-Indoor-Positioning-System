/*
*  @file    uart_priv.h
*  @brief   Method1 - TOA: Source Code for Sync Node with Clock Correction extension
*  @author 	Stefan Koller, BSc
*/

#include "uart_pub.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_uart.h"
#include "bsp.h"
#include "boards.h"

#define NO_PARITY	false

// UART circular buffers - Tx and Rx size
#define UART_TX_BUF_SIZE 512
#define UART_RX_BUF_SIZE 32

// UART initialisation structure
const app_uart_comm_params_t comm_params_UART =
{
    RX_PIN_NUM,
    TX_PIN_NUM,
    RTS_PIN_NUM,
    CTS_PIN_NUM,
    APP_UART_FLOW_CONTROL_DISABLED,
    NO_PARITY,
    NRF_UART_BAUDRATE_115200
};

// local function prototypes
static void vHandleUartInternalErrors (uint32_t u32Error);
static void vUartErrorHandle					(app_uart_evt_t * p_event);
