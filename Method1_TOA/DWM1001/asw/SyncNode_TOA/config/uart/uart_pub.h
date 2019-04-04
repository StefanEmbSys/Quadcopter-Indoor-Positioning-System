/*
*  @file    uart_pub.h
*  @brief   Method1 - TOA: Source Code for Sync Node
*  @author 	Stefan Koller, BSc
*/

#include <stdbool.h>
#include <stdint.h>

// Prototypes of extern functions
void UART_Init	(void);
bool UART_getc	(uint8_t *u8ch);
