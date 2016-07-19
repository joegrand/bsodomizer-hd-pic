/*** FILEHEADER ****************************************************************
 *
 *    FILENAME:    RC5_Decoder.c
 *
 *    DESCRIPTION: RC5 Decoder driver
 *
 ******************************************************************************/

#include <xc.h>
#include "RC5_Decoder.h"

/* RC5 Decoding status variables */
static uint16 RC5_DECODER_code = 0;
static uint8 RC5_DECODER_timeoutTimer = 0; // Timeout timer, unit [ms]
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
    //OPTION_REG = 0x06; // Global Pull ups enabled, TMR0 = ON, 1:128 prescaler
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
/*       because of decoding state machine stucks due to erroneous RC5       */
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
    static uint8 rc5_pos = 13;
    static uint16 rc5_code_tmp = 0;
    static bool rc5_wait_start = TRUE;
    static bool rc5_bit_state = RC5_BIT_STATE_FULL;
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

    if (rc5_wait_start != FALSE) {
        /* 1st half of start bit received */
        if ((rc5_pin_old != FALSE) && (rc5_pin == FALSE)) {
            /* leave wait state */
            rc5_wait_start = FALSE;
            /* 1st half of bit has been received */
            rc5_bit_state = RC5_BIT_STATE_HALF;
            /* 1st start bit is at position 13 */
            rc5_pos = 13;
            /* reset RC5 code */
            rc5_code_tmp = 0;
        }
    } else {
        /* rising edge of RC5 signal */
        if ((rc5_pin_old == FALSE) && (rc5_pin != FALSE)) {
            /* one half of the bit has already been received last time and now */
            /* we got the 2nd half of the bit */
            if ((rc5_bit_state == RC5_BIT_STATE_HALF) && (tdiff >= RC5_HALF_BIT_TIME_MIN) && (tdiff <= RC5_HALF_BIT_TIME_MAX)) {
                /* logical 1 has been received => add to rc5 code */
                bit_set(rc5_code_tmp, rc5_pos);
                /* decrement bit position */
                if (rc5_pos > 0) {
                    rc5_pos--;
                } else {
                    rc5_rx_last = TRUE;
                }
                /* we are at the end of a bit */
                rc5_bit_state = RC5_BIT_STATE_FULL;
            }   /* one half of the bit has already been received last time and now */
                /* we got the 2nd half of the bit and also the next half of the following bit */
            else if ((rc5_bit_state == RC5_BIT_STATE_HALF) && (tdiff >= (2 * RC5_HALF_BIT_TIME_MIN)) && (tdiff <= (2 * RC5_HALF_BIT_TIME_MAX))) {
                /* logical 1 has been received => add to rc5 code */
                //rc5_code_tmp |= (((uint16)1 << rc5_pos));
                //rc5_code_tmp |= ((uint16)(1 << rc5_pos));
                bit_set(rc5_code_tmp, rc5_pos);
                /* decrement bit position */
                if (rc5_pos > 0) {
                    rc5_pos--;
                }

                /* not done in else part because there will be no additional interrupt for the last bit in this case */
                if (rc5_pos == 0) {
                    rc5_rx_last = TRUE;
                }

                /* we are at the half of the next bit */
                rc5_bit_state = RC5_BIT_STATE_HALF;
            }                /* previously, we have sampled a full bit, since now only the 1st */
                /* half of the next bit is available now, wait until the next half */
            else if ((rc5_bit_state == RC5_BIT_STATE_FULL) && (tdiff >= RC5_HALF_BIT_TIME_MIN) && (tdiff <= RC5_HALF_BIT_TIME_MAX)) {
                /* we are at the half of the next bit */
                rc5_bit_state = RC5_BIT_STATE_HALF;

                /* we will not get an interrupt for the last half bit, so we are finished here */
                if (rc5_pos == 0) {
                    rc5_rx_last = TRUE;
                }
            }                /* something wrong with the timing, wait for another start condition */
            else {
                /* wait for start condition */
                rc5_wait_start = TRUE;
            }
        }
        /* falling edge of RC5 signal */
        if ((rc5_pin_old != FALSE) && (rc5_pin == FALSE)) {
            /* one half of the bit has already been received last time and now */
            /* we got the 2nd half of the bit */
            if ((rc5_bit_state == RC5_BIT_STATE_HALF) && (tdiff >= RC5_HALF_BIT_TIME_MIN) && (tdiff <= RC5_HALF_BIT_TIME_MAX)) {
                /* logical 0 has been received => add to rc5 code */
                bit_clear(rc5_code_tmp, rc5_pos);
                /* decrement bit position */
                if (rc5_pos > 0) {
                    rc5_pos--;
                } else {
                    rc5_rx_last = TRUE;
                }
                /* we are at the end of a bit */
                rc5_bit_state = RC5_BIT_STATE_FULL;
            }                /* one half of the bit has already been received last time and now */
                /* we got the 2nd half of the bit and also the next half of the following bit */
            else if ((rc5_bit_state == RC5_BIT_STATE_HALF) && (tdiff >= (2 * RC5_HALF_BIT_TIME_MIN)) && (tdiff <= (2 * RC5_HALF_BIT_TIME_MAX))) {
                /* logical 0 has been received => add to rc5 code */
                //rc5_code_tmp |= (0 << rc5_pos);
                bit_clear(rc5_code_tmp, rc5_pos);
                /* decrement bit position */
                if (rc5_pos > 0) {
                    rc5_pos--;
                } else {
                    rc5_rx_last = TRUE;
                }
                /* we are at the half of the next bit */
                rc5_bit_state = RC5_BIT_STATE_HALF;
            }                /* previously, we have sampled a full bit, since now only the 1st */
                /* half of the next bit is available now, wait until the next half */
            else if ((rc5_bit_state == RC5_BIT_STATE_FULL) && (tdiff >= RC5_HALF_BIT_TIME_MIN) && (tdiff <= RC5_HALF_BIT_TIME_MAX)) {
                /* we are at the half of the next bit */
                rc5_bit_state = RC5_BIT_STATE_HALF;
            }                /* something wrong with the timing, wait for another start condition */
            else {
                /* wait for start condition */
                rc5_wait_start = TRUE;
            }
        }
    }

    /* save current pin state for edge detection */
    rc5_pin_old = rc5_pin;

    /* the first two bits are the start bits and have to be 1, else something */
    /* went wrong and hence wait for another start condition */
    if (((rc5_pos == 12) && !bit_test(rc5_code_tmp, RC5_START_BIT_1)) ||
            ((rc5_pos == 11) && !bit_test(rc5_code_tmp, RC5_START_BIT_2))) {
        rc5_wait_start = TRUE;
    }

    /* if all RC5 bits have been received */
    if (rc5_pos == 0) {
        RC5_Decode.valid = TRUE; // Signal that valid RC5 data was loaded
    }
    if ((rc5_pos == 0) && (rc5_rx_last != FALSE)) {
        /* if code is different from last received code, set flag to indicate */
        /* that a new code has been received */
        if (RC5_DECODER_code != rc5_code_tmp) {
            RC5_DECODER_code = rc5_code_tmp;
            RC5_DECODE_ready = TRUE;
        }
        rc5_wait_start = TRUE;
    }
}
