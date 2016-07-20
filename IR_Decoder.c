//
// BSODomizer HD
//
// Filename: 	IR_Decoder.c
// Author: 	  Joe Grand [www.grandideastudio.com] and Zoz
//
// Description: IR (NEC/Apple Remote) Decoder
//
// Distributed under a Creative Commons Attribution 3.0 United States license
// http://creativecommons.org/licenses/by/3.0/us/ 
//
// Protocol details from:
// http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
// https://en.wikipedia.org/wiki/Apple_Remote
// https://hifiduino.wordpress.com/apple-aluminum-remote/
//

#include <XC.h>
#include "IR_Decoder.h"

/* RC5 Decoding status variables */
static uint32 RC5_DECODER_code = 0;
static uint16 RC5_DECODER_timeoutTimer = 0; // Timeout timer, unit [ms]
static bool RC5_DECODE_ready = FALSE;
volatile RC5_DECODE RC5_Decode; // Global structure to store RC5 Decoder data

/*****************************************************************************/
/* RC5_DECODER_init                                                          */
/*                                                                           */
/* Call once at startup                                                      */
/*                                                                           */
/* Return:  Nothing                                                          */

/*****************************************************************************/
void RC5_DECODER_init(void) {
    uint8 dummy;
    
    RC5_DECODER_TRISx = 1; // Configure as input
    RC5_DECODER_ANSx = 0; // Disable analog function
    RC5_DECODER_IOCxP = 1; // Enable positive edge detect
    RC5_DECODER_IOCxN = 1; // Enable negative edge detect
    RC5_DECODER_WPUx = 1; // Enable weak pull-up
    //OPTION_REG = 0x06; // Global Pull ups enabled, TMR0 = ON, 1:128 prescaler for 16MHz Fosc
    dummy = RC5_GetPin(); // Clear mismatch
    RC5_DECODER_IOCxF = 0; // Clear IOCx interrupt flag
}

/*****************************************************************************/
/* RC5_CodeReady                                                             */
/*                                                                           */
/* Returns status if a new RC5 code has been received.                       */
/*                                                                           */
/* Return:  TRUE if new RC5 code is available, FALSE otherwise.              */

/*****************************************************************************/
bool RC5_DECODER_codeReady(void) {
    bool rc;
    rc = RC5_DECODE_ready;
    RC5_DECODE_ready = FALSE;
    return (rc);
}

/*****************************************************************************/
/* RC5_GetCode                                                               */
/*                                                                           */
/* Returns received RC5 code. Use RC5_CodeReady() function first to see if   */
/* a valid code is available.                                                */
/*                                                                           */
/* Return:  rc5_code    RC5 code.                                            */

/*****************************************************************************/
uint16 RC5_DECODER_getCode(void) {
    return (RC5_DECODER_code);
}

/*****************************************************************************/
/* RC5_TimeoutIncrement                                                      */
/*                                                                           */
/* Function increments timeout counter to detect a RC5 timeout and           */
/* resynchronize the RC5 decoding state machine.                             */
/* Functions shall be called cyclically in main loop or in a timer           */
/* interrupt overflow routine which is called at around every 1ms.           */
/*                                                                           */
/* NOTE: Decoding will also work without calling this function, but          */
/*       it could happen that RC5 codes are sometimes not getting recognized */
/*       because of decoding state machine sticks due to erroneous RC5       */
/*       signal.                                                             */
/*                                                                           */
/* Return:  none                                                             */

/*****************************************************************************/
void RC5_DECODER_timeoutIncrement(void) {
    static uint8 old_timer;
    uint8 timer;

    /* get current timer value */
    timer = RC5_GetTimer();

    /* disable interrupts since rc5_timeout_timer is also read and written to in ISR */
    disable_interrupts(GLOBAL);
    //INTCONbits.GIE = 0;

    if (RC5_DECODER_timeoutTimer < RC5_TIMEOUT) {
        RC5_DECODER_timeoutTimer += (timer - old_timer);
    }

    /* re-enable interrupts again */
    enable_interrupts(GLOBAL);
    //INTCONbits.GIE = 1;

    old_timer = timer;
}

/*****************************************************************************/
/* RC5_InterruptHandler                                                      */
/*                                                                           */
/* Interrupt handler for RC5 decoding. This function must be called by an    */
/* "Interrupt-On-Change" interrupt service routine.                          */
/*                                                                           */
/* Return:  none                                                             */

/*****************************************************************************/
void RC5_DECODER_interruptHandler(void) {
    static uint8 rc5_timer = 0;
    static uint8 rc5_pos = 0;
    static uint32 rc5_code_tmp = 0;
    static bool rc5_wait_start = TRUE;
    static bool rc5_wait_space = TRUE;
    //static bool rc5_bit_state = RC5_BIT_STATE_FULL;
    static bool rc5_pin_old = TRUE;
    bool rc5_rx_last = FALSE;
    bool rc5_pin;
    uint8 tdiff;

    /* Signal that RC5 data is not valid (yet) */
    RC5_Decode.valid = FALSE;

    /* get RC5 pin status */
    rc5_pin = RC5_GetPin();

    /* calculate time difference to last interrupt call */
    tdiff = RC5_GetTimer() - rc5_timer;

    /* start the RC5 timer again */
    rc5_timer = RC5_GetTimer();

    /* if timeout counter has expired, i.e. no RC5 signal was received for some */
    /* time, reset the state machine */
    if (RC5_DECODER_timeoutTimer >= RC5_TIMEOUT) {
        rc5_wait_start = TRUE;
    }
    /* reset the timeout counter */
    RC5_DECODER_timeoutTimer = 0;

    if ((rc5_wait_start != FALSE) && (rc5_pin_old == FALSE) && (rc5_pin != FALSE)) {     // wait for the header, rising edge
      if (tdiff >= RC5_PREPULSE)
      {
            rc5_wait_start = FALSE; // leave wait state
      }
    }
    else if ((rc5_wait_start == FALSE) && (rc5_wait_space != FALSE)) { // wait for the space
      {
        if (tdiff >= RC5_SPACE){
          rc5_wait_space = FALSE;
          RC5_DECODER_code = 0;
          rc5_pos = 0;
        }
        else
          rc5_wait_start = TRUE;
      }
    }
    else if ((rc5_wait_start == FALSE) && (rc5_wait_space == FALSE)) {   // start receiving the actual data
        /* rising edge of RC5 signal */
        if ((rc5_pin_old == FALSE) && (rc5_pin != FALSE)) {
          //rc5_timer = RC5_GetTimer();
        }
        /* falling edge of RC5 signal */
        if ((rc5_pin_old != FALSE) && (rc5_pin == FALSE)) {
          rc5_pos++;
          RC5_DECODER_code <<= 1;
          if (tdiff >= RC5_BIT)
            RC5_DECODER_code |= 1;
        }
    }

    /* save current pin state for edge detection */
    rc5_pin_old = rc5_pin;

    /* if all RC5 bits have been received */
    if (rc5_pos == 32) {
        RC5_Decode.valid = TRUE; // Signal that valid RC5 data was loaded
        rc5_wait_start = TRUE;
        rc5_wait_space = TRUE;
        rc5_pos = 0;
    }
}
