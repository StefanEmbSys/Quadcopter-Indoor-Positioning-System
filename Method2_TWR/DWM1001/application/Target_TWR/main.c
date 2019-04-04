/*
*  @file    main.c
*  @brief   Method2 - TWR: Source Code for Target
*  @author 	Stefan Koller, BSc
*/


#include "sdk_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_uart.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "app_error.h"
#include <string.h>
#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "uart.h"
#include "dw1000.h"
	

/* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
#define PRE_TIMEOUT 1000

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100 

/*Should be accurately calculated during calibration*/
#define TX_ANT_DLY 16300
#define RX_ANT_DLY 16456	

#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      2000          /**< Timer period. LED1 timer will expire after 1000 ms */

#ifdef USE_FREERTOS

TaskHandle_t  ss_initiator_task_handle;   /**< Reference to SS TWR Initiator FreeRTOS task. */
extern void target_task_function (void * pvParameter);
#endif

int main(void)
{
  /* Setup the custom LEDs */
  LEDS_CONFIGURE(LEDS_MASK);
  LEDS_OFF(LEDS_MASK);
  //LEDS_ON(BSP_LED_0_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_1_MASK (D10), BSP_LED_2_MASK (D11), BSP_LED_3_MASK (D12), LEDS_MASK (D9-D12))   
  
  #ifdef USE_FREERTOS
    /* Create task for SS TWR Initiator set to 2 */
    UNUSED_VARIABLE(xTaskCreate(target_task_function, "Target", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ss_initiator_task_handle));
  #endif // #ifdef USE_FREERTOS
  
  /*Initialization UART*/
  boUART_Init ();
  printf("DWM1001 module used as Target.\r\n");

  // DW1000 initialization
  dw1000_init();
  
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
