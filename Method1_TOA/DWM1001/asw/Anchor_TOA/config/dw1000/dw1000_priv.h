/*
*  @file    dw1000_priv.h
*  @brief   Method1 - TOA: Source Code for Anchor (Base Station)
*  @author 	Stefan Koller, BSc
*/

#include "dw1000/dw1000_pub.h"
#include "boards.h"
#include "port_platform.h"
//#include "deca_device_api.h"
//#include "deca_types.h"
//#include "deca_param_types.h"
//#include "deca_regs.h"


//-----------------Config for DW1000 module ----------------------------
static dwt_config_t config_DW1000 = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
#define PRE_TIMEOUT 1000

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100 

/*Should be accurately calculated during calibration*/
#define TX_ANT_DLY 16300
#define RX_ANT_DLY 16456	