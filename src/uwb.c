#include "pt.h"
#include "deca_device_api.h"

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

void init_uwb() {
    while (!dwt_checkidlerc()) {}
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("uwb error\n");
    }
    
    // dwt_config_t config = {
    //     5,               /* Channel number. */
    //     DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    //     DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    //     9,               /* TX preamble code. Used in TX only. */
    //     9,               /* RX preamble code. Used in RX only. */
    //     1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    //     DWT_BR_6M8,      /* Data rate. */
    //     DWT_PHRMODE_STD, /* PHY header mode. */
    //     DWT_PHRRATE_STD, /* PHY header rate. */
    //     (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    //     DWT_STS_MODE_OFF, /* STS disabled */
    //     DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    //     DWT_PDOA_M0      /* PDOA mode off */
    // };
    // dwt_configure(&config);

    // dwt_configuretxrf(&txconfig_options);

    // dwt_setrxantennadelay(RX_ANT_DLY);
    // dwt_settxantennadelay(TX_ANT_DLY);

    // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    // dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    
}