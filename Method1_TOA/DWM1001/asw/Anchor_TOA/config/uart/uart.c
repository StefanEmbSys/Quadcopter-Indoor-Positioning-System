/*
*  @file    uart.c
*  @brief   Method1 - TOA: Source Code for Anchor (Base Station)
*  @author 	Stefan Koller, BSc
*/

#include "uart_priv.h"

void UART_Init(void)
{
    // Initialis the nrf UART driver returning state
    uint32_t err_code;
    APP_UART_FIFO_INIT (&comm_params_UART, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, vUartErrorHandle, APP_IRQ_PRIORITY_LOWEST, err_code);
    if (err_code == NRF_SUCCESS)
    {
      printf("UART init OK \r\n");
    }
    else
    {
      printf("UART init ERROR \r\n");
    }
}

bool UART_getc(uint8_t *u8ch)
{
    bool Success = false;
    if (app_uart_get(u8ch) == NRF_SUCCESS)
    {
      Success = true;
    }
    return Success;
}

static void vUartErrorHandle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        vHandleUartInternalErrors(p_event->evt_type);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        vHandleUartInternalErrors(p_event->evt_type);
    }
}

static void vHandleUartInternalErrors (uint32_t u32Error)
{
    // notify app of error - LED ?
}

