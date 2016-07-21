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
// Original structure and inspiration:
// http://www.embeddedcodesource.com/codesnippet/ir-remote-control-philips-rc5-
// protocol-encoder-decoder

// NEC/Apple Remote protocol details:
// http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
// https://en.wikipedia.org/wiki/Apple_Remote
// https://hifiduino.wordpress.com/apple-aluminum-remote/
//

#include <XC.h>
#include <stdint.h>
#include "BSOD_Main.h"
#include "IR_Decoder.h"

/* NEC Decoding status variables */
static uint32_t NEC_DECODER_code = 0;
static uint16_t NEC_DECODER_timeoutTimer = 0; // Timeout timer, unit [ms]
static uint8_t NEC_DECODE_ready = FALSE;
volatile APPLE_DECODE NEC_Decode; // structure to store NEC Decoder data


/*****************************************************************************/
/* NEC_DECODER_init                                                          */
/*                                                                           */
/* Call once at startup                                                      */
/*                                                                           */
/* Return:  Nothing                                                          */

/*****************************************************************************/
void NEC_DECODER_init(void) {
    uint8_t dummy;
    
    NEC_DECODER_TRISx = 1;  // Configure as input
    NEC_DECODER_ANSx = 0;   // Disable analog function
    NEC_DECODER_IOCxP = 1;  // Enable positive edge detect
    NEC_DECODER_IOCxN = 1;  // Enable negative edge detect
    NEC_DECODER_WPUx = 1;   // Enable weak pull-up
    //OPTION_REG = 0x06; // Global Pull ups enabled, TMR0 = ON, 1:128 prescaler for 16MHz Fosc
    dummy = NEC_GetPin();   // Clear mismatch
    NEC_DECODER_IOCxF = 0;  // Clear IOCx interrupt flag
}


/*****************************************************************************/
/* NEC_TimeoutIncrement                                                      */
/*                                                                           */
/* Function increments timeout counter to detect a NEC timeout and           */
/* resynchronize the NEC decoding state machine.                             */
/* Functions shall be called cyclically in main loop or in a timer           */
/* interrupt overflow routine which is called at around every 1ms.           */
/*                                                                           */
/* NOTE: Decoding will also work without calling this function, but          */
/*       it could happen that NEC codes are sometimes not getting recognized */
/*       because of decoding state machine sticks due to erroneous NEC       */
/*       signal.                                                             */
/*                                                                           */
/* Return:  none                                                             */
/*****************************************************************************/
void NEC_DECODER_timeoutIncrement(void) {
    static uint8_t old_timer;
    uint8_t timer;

    /* get current timer value */
    timer = NEC_GetTimer();

    /* disable interrupts since nec_timeout_timer is also read and written to in ISR */
    GIE = 0;

    if (NEC_DECODER_timeoutTimer < NEC_TIMEOUT) 
    {
        NEC_DECODER_timeoutTimer += (timer - old_timer);
    }

    /* re-enable interrupts again */
    GIE = 1;

    old_timer = timer;
}


/*****************************************************************************/
/* NEC_InterruptHandler                                                      */
/*                                                                           */
/* Interrupt handler for NEC decoding. This function must be called by an    */
/* "Interrupt-On-Change" interrupt service routine.                          */
/*                                                                           */
/* Return:  none                                                             */

/*****************************************************************************/
void NEC_DECODER_interruptHandler(void) 
{
    static uint8_t nec_timer = 0;
    static uint8_t nec_pos = 0;
    static uint8_t nec_pin_old = TRUE;
    uint8_t nec_pin;
    uint8_t tdiff;
    static uint8_t NEC_DECODE_state = STATE_WAIT_PREPULSE;

    /* Signal that Apple remote data is not valid (yet) */
    NEC_Decode.valid = FALSE;

    /* get NEC pin status */
    nec_pin = NEC_GetPin();

    /* calculate time difference to last interrupt call */
    tdiff = NEC_GetTimer() - nec_timer;

    /* start the RC5 timer again */
    nec_timer = NEC_GetTimer();

    /* if timeout counter has expired, i.e. no NEC signal was received for some */
    /* time, reset the state machine */
    if (NEC_DECODER_timeoutTimer >= NEC_TIMEOUT) 
    {
        NEC_DECODE_state = STATE_WAIT_PREPULSE;
    }
    /* reset the timeout counter */
    NEC_DECODER_timeoutTimer = 0;

    switch (NEC_DECODE_state) 
    {
      case STATE_WAIT_PREPULSE:
        if ((nec_pin_old == FALSE) && (nec_pin != FALSE)) // if a rising edge (end of prepulse)
        {     
          if (tdiff >= NEC_PREPULSE)
          {
            NEC_DECODE_state = STATE_WAIT_SPACE;
          }
        }
        break;
      case STATE_WAIT_SPACE:
        if (tdiff >= NEC_SPACE) // if the space is the correct length
        {	
          NEC_DECODE_state = STATE_READING_DATA;
          NEC_DECODER_code = 0;
          nec_pos = 0;
         }
         else // otherwise go back to waiting for the prepulse
         {		
           NEC_DECODE_state = STATE_WAIT_PREPULSE;
         }
         break;
      case STATE_READING_DATA:
         /* do nothing on rising edge of NEC signal */
         /* clock in bits on falling edge of NEC signal */
         if ((nec_pin_old != FALSE) && (nec_pin == FALSE)) 
         {
           nec_pos++;
           NEC_DECODER_code <<= 1;
           if (tdiff >= NEC_BIT) // long pulses are ones; otherwise low bit is already zero
           {	
             NEC_DECODER_code |= 1;
           }
         }
         break;
    }

    /* save current pin state for edge type detection */
    nec_pin_old = nec_pin;

    /* if all NEC bits have been received */
    if (nec_pos == 32) 
    {
        NEC_Decode.valid = TRUE; // Signal that valid NEC data was loaded
        NEC_Decode.command = NEC_DECODER_getCmd(NEC_DECODER_code); // Populate command from Apple Remote (ignore the rest of the bits)
        NEC_DECODE_state = STATE_WAIT_PREPULSE;
    }
}

uint8_t hasValidDecode(void)
{
  return (NEC_Decode.valid);
}

apple_cmd_type getAppleCommand(void)
{
  return (NEC_Decode.command);
}

void resetDecode(void)
{
  NEC_Decode.valid = FALSE;
}
