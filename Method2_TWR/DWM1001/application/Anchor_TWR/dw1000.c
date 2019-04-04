/*
*  @file    dw1000.c
*  @brief   Method2 - TWR: Source Code for Anchor Nodes
*  @author 	Stefan Koller, BSc
*/

  #include "dw1000.h"
  
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
      while (1)
      {};
    }

    // Set SPI to 8MHz clock
    port_set_dw1000_fastrate();  

    /* Configure DW1000. */
    dwt_configure(&config_DW1000);

    /* Apply default antenna delay value. Defined in port platform.h */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set preamble timeout for expected frames.  */
    //dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    dwt_setrxtimeout(0);    // set to NO receive timeout for this simple example 
  }
