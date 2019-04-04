/*
*  @file    dw1000_pub.h
*  @brief   Method1 - TOA: Source Code for Sync Node
*  @author 	Stefan Koller, BSc
*/

#include <stdbool.h>

/*Interrupt flag*/
static volatile int tx_int_flag; // Transmit success interrupt flag
static volatile int rx_int_flag; // Receive success interrupt flag
static volatile int to_int_flag; // Timeout interrupt flag
static volatile int er_int_flag; // Error interrupt flag 

// Prototypes of extern functions
void dw1000_init();
