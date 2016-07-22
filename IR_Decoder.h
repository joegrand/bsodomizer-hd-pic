//
// BSODomizer HD
//
// Filename: 	IR_Decoder.h
// Author: 	  Joe Grand [www.grandideastudio.com] and Zoz
//
// Description: IR (NEC/Apple Remote) Decoder header file
//
// Distributed under a Creative Commons Attribution 3.0 United States license
// http://creativecommons.org/licenses/by/3.0/us/ 
//

#include <stdint.h>

#ifndef __BSOD_IR_H__
#define __BSOD_IR_H__

/*****************************************************************************/
/* DRIVER CONFIGURATION                                                      */
/* ====================                                                      */
/*                                                                           */
/* The following defines need to be defined by the user.                     */
/*                                                                           */
/* The following configuration items are mandatory:                          */
/* - NEC_DATA_PIN:      NEC input pin                                        */
/* - NEC_TICKS_PER_MS:  NEC timer ticks per millisecond                      */
/* - NEC_GetTimer():    Macro to get NEC timer value                         */
/*                                                                           */
/*****************************************************************************/

// Apple remote receiver sensor connection
#define NEC_DATA_PIN          RB4
#define NEC_DECODER_TRISx     TRISB4
#define NEC_DECODER_ANSx      ANSB4
#define NEC_DECODER_WPUx      WPUB4
#define NEC_DECODER_IOCxP     IOCBP4
#define NEC_DECODER_IOCxN     IOCBN4
#define NEC_DECODER_IOCxF     IOCBF4

// Timer 0 is configured to increment every 128.0us (i.e. 1000/128 times per ms)
#define NEC_TICKS_PER_MS      (1000/128)

// Timer 0 used for time measurement of RC5 signal
#define NEC_GetTimer()        TMR0

// Decoding states
#define STATE_WAIT_PREPULSE   0x00
#define STATE_WAIT_SPACE      0x01
#define STATE_READING_DATA    0x02


/**************************************************************************
************************** Typedefs ***************************************
***************************************************************************/

typedef enum	// 7-bit command 
{
  APPLE_UP      = 0x05,
  APPLE_DOWN    = 0x06,
  APPLE_LEFT    = 0x04,
  APPLE_RIGHT   = 0x03,
  APPLE_CENTER  = 0x02,
  APPLE_MENU    = 0x01,   
} apple_cmd_type;

/*--- Apple remote data structure ---*/

typedef struct {
    apple_cmd_type command;
    uint8_t valid;
} APPLE_DECODE;


/*****************************************************************************/
/* NEC timings                                                               */
/*****************************************************************************/

#define NEC_GetPin()            NEC_DATA_PIN

#define NEC_PREPULSE            ((NEC_TICKS_PER_MS) * 8)  // Leading pulse burst is 9ms per NEC spec
#define NEC_SPACE               ((NEC_TICKS_PER_MS) * 4)  // Following space before data is 4.5ms per NEC spec
#define NEC_BIT                 ((NEC_TICKS_PER_MS))      // Time threshold to determine a '0' (562.5us space) or '1' (1.6875ms space) 
#define NEC_TIMEOUT             ((NEC_TICKS_PER_MS) * 2)  // Time threshold for too long of a data bit


/*****************************************************************************/
/* NEC_DECODER_getCmd                                                        */
/*                                                                           */
/* Extract the command (7 bits) from the full Apple Remote code              */
/*                                                                           */
/* Example for white Apple Remote Menu button:                               */
/* 0x77E1C064 = 0111011111100001 1 1000000 01100100                          */
/*                                 \-----/                                   */
/*                                 Command                                   */
/* MSB is on the right, so this macro will flip the bits                     */
/*****************************************************************************/

static unsigned char lookup[16] = {0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf};
#define NEC_MASK_CMD_LOWNIBBLE 0x00000F00
#define NEC_MASK_CMD_HIGHNIBBLE 0x00007000
#define NEC_DECODER_getCmd(neccode) ((lookup[(neccode & NEC_MASK_CMD_LOWNIBBLE)>>8] << 4) | (lookup[(neccode & NEC_MASK_CMD_HIGHNIBBLE)>>12]) >> 1)


void NEC_DECODER_init(void);
uint8_t NEC_DECODER_codeReady(void);
void NEC_DECODER_interruptHandler(void);

/*****************************************************************************/
/* External use functions                                                    */
/*****************************************************************************/
uint8_t hasValidDecode(void);
apple_cmd_type getAppleCommand(void);
void resetDecode(void);

#endif /* __BSOD_IR_H__ */