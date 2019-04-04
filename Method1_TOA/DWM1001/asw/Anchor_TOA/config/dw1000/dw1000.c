/*
*  @file    dw1000.c
*  @brief   Method1 - TOA: Source Code for Anchor (Base Station)
*  @author 	Stefan Koller, BSc
*/
  #include "dw1000/dw1000_priv.h"
  
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
