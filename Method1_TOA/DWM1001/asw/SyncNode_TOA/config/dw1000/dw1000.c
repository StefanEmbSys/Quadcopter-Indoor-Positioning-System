/*
*  @file    dw1000.c
*  @brief   Method1 - TOA: Source Code for Sync Node
*  @author 	Stefan Koller, BSc
*/

  #include "dw1000/dw1000_priv.h"

void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  rx_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #1 */
}

void rx_to_cb(const dwt_cb_data_t *cb_data)
{
  to_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #2 */
  printf("TimeOut\r\n");
}

void rx_err_cb(const dwt_cb_data_t *cb_data)
{
  er_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #3 */
  printf("Transmission Error : may receive package from different UWB device\r\n");
}

void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
  /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
  * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
  * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
  * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
  * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

  tx_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #4 */
}
  
  void dw1000_init()
  {
      /* Setup DW1000 IRQ pin */  
      nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 		//irq
  
      /* Reset DW1000 */
      reset_DW1000(); 

      /* Set SPI clock to 2MHz */
      port_set_dw1000_slowrate();			

      /* Init the DW1000 */
      if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
      {
        //Init of DW1000 Failed
        printf("Init of DW1000 failed. \r\n");
        while (1) {};
      }

      // Set SPI to 8MHz clock
      port_set_dw1000_fastrate();

      /* Configure DW1000. */
      dwt_configure(&config_DW1000);

      tx_int_flag = 0;
      rx_int_flag = 0;
      to_int_flag = 0;
      er_int_flag = 0;

      /* Initialization of the DW1000 interrupt*/
      /* Callback are defined in ss_init_main.c */
      //dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

      /* Apply default antenna delay value. See NOTE 2 below. */
      dwt_setrxantennadelay(RX_ANT_DLY);
      dwt_settxantennadelay(TX_ANT_DLY);

      /* Set preamble timeout for expected frames. See NOTE 3 below. */
      //dwt_setpreambledetecttimeout(0); // PRE_TIMEOUT
        
      /* Set expected response's delay and timeout. 
      * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
      dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
      dwt_setrxtimeout(65000); // Maximum value timeout with DW1000 is 65ms  
  }
