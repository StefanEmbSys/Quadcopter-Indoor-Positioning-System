/*
*  @file    main.c
*  @brief   Method1 - TOA: Source Code for Sync Node with Clock Correction extension
*  @author 	Stefan Koller, BSc
*/

//#include "sdk_config.h"
//#include "bsp.h"
//#include "boards.h"
//#include "nordic_common.h"
//#include "nrf_drv_clock.h"
//#include "nrf_drv_spi.h"
//#include "nrf_uart.h"
//#include "app_util_platform.h"
//#include "nrf_gpio.h"
//#include "nrf_delay.h"
//#include "nrf_log.h"
//#include "nrf.h"
//#include "app_error.h"
//#include "app_util_platform.h"
//#include "app_error.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "port_platform.h"
#include "app_uart.h"
#include "uart/uart_pub.h"
#include "dw1000/dw1000_pub.h"
#include "syncnode.h"

//#define TIMER_PERIOD      10655          // Timer period. Timer will expire after approx. 10000 ms
//#define TIMER_PERIOD       1065          // Timer period. Timer will expire after approx. 1000 ms
//#define TIMER_PERIOD          213          // Timer period. Timer will expire after approx. 200 ms
//#define TIMER_PERIOD          106          // Timer period. Timer will expire after approx. 100 ms
//#define TIMER_PERIOD          103          // Timer period. Timer will expire after approx. 100 ms	
#define TIMER_PERIOD          10          // Timer period. Timer will expire after approx. ? ms

#ifdef USE_FREERTOS
TimerHandle_t SyncPeriod_timer_handle;
#endif

int main(void)
  {
  /* Setup the custom LEDs */
  LEDS_CONFIGURE(LEDS_MASK);
  LEDS_OFF(LEDS_MASK);
  //LEDS_ON(BSP_LED_0_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_1_MASK (D10), BSP_LED_2_MASK (D11), BSP_LED_3_MASK (D12), LEDS_MASK (D9-D12))   

  #ifdef USE_FREERTOS
    /* Start timer for Sync-Transmission */
    SyncPeriod_timer_handle = xTimerCreate( "SyncPeriod", TIMER_PERIOD, pdTRUE, NULL, SyncPeriod_timer_callback);
    UNUSED_VARIABLE(xTimerStart(SyncPeriod_timer_handle, 0));
  #endif // #ifdef USE_FREERTOS

  printf("SyncNode \r\n");

  // DW1000 initialization
  dw1000_init();

  // UART initialization
  UART_Init();

  // SyncNode initialization
  ini_SyncNode();

  // Fill Tx_Sync message
  fill_fixed_txSync();

  // Wait for "GO" from ATLAS
  wait_for_GO();

  #ifdef USE_FREERTOS		
    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();	

    while(1) 
    {};
  #else

    // No RTOS task here so just call the main loop here.
    // Loop forever responding to ranging requests.
    while (1)
    {
      ss_init_run();
    }

  #endif
}
