/*
*  @file    main.c
*  @brief   Method2 - TWR: Source Code for Anchor Nodes
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
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf.h"
#include "app_error.h"
#include <string.h>
#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "dw1000.h"


#define TASK_DELAY        10            /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      2000          /**< Timer period. LED1 timer will expire after 1000 ms */

#ifdef USE_FREERTOS
  TaskHandle_t  anchor_task_handle;   /**< Reference to SS TWR Initiator FreeRTOS task. */
  extern void anchor_task_function (void * pvParameter);
#endif    // #ifdef USE_FREERTOS


int main(void)
{
  /* Setup the custom LEDs */
  LEDS_CONFIGURE(LEDS_MASK);
  LEDS_OFF(LEDS_MASK);
  //LEDS_ON(BSP_LED_0_MASK);         // BSP_LED_0_MASK (D9), BSP_LED_3_MASK (D12), BSP_LED_2_MASK (D11), BSP_LED_1_MASK (D10), LEDS_MASK (D9-D12))   

  #ifdef USE_FREERTOS
    /* Create task for SS TWR Initiator set to 2 */
    UNUSED_VARIABLE(xTaskCreate(anchor_task_function, "Anchor", configMINIMAL_STACK_SIZE + 200, NULL, 2, &anchor_task_handle)); 
  #endif  // #ifdef USE_FREERTOS

  printf("DW1000 module used as Anchor \r\n");

  // DW1000 initialization
  dw1000_init();

  #ifdef USE_FREERTOS
    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();	

    while(1)
    {}
  #else
    // No RTOS task here so just call the main loop here.
    // Loop forever responding to ranging requests.
    while (1)
    {
      ss_resp_run();
    }
    #endif  // #ifdef USE_FREERTOS
}
