//
// BSODomizer HD
// Copyright (c) 2016 Grand Idea Studio, Inc.
//
// Filename: 	BSOD_Main.c
// Author: 	  Joe Grand [www.grandideastudio.com]
//
// Description: Main file for the BSODomizer HD Front End Subsystem
// Processor: 	Microchip PIC16LF1828
// Compiler: 	  MPLAB X IDE v3.35 w/ Microchip XC8 v1.38 (Free mode)
//
// Distributed under a Creative Commons Attribution 3.0 United States license
// http://creativecommons.org/licenses/by/3.0/us/ 
//

#include "BSOD_Main.h"
#include "RC5_Decoder.h"


/****************************************************************************
 ************************** Global variables ********************************
 ***************************************************************************/
 
volatile mode_type gMode = START_UP;

// Clock/Timer
volatile uint16_t gCounter = 0;   // Counter (incremented on every Timer 1 interrupt)

struct time_struct
{
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} gClock = {0, 0, 0};  // Current time

// ADC
uint8_t adc_vbat = 42;  // Current battery voltage (in 0.1V increments)

// Flags
volatile button_state_type 	gSW = SW_NONE;   // state of buttons (debounced)


/****************************************************************************
 ********************** Interrupt Service Routines **************************
 ***************************************************************************/

void interrupt isr(void)
{
  if (TMR1IF == 1)  // Timer 1: Accurate clock/timing (interrupt every 2 seconds)
	{
    TMR1H = TMR1H_LOAD; // Reload timer 1 overflow time
    TMR1L = TMR1L_LOAD;
		TMR1IF = 0;         // Clear the interrupt flag
    ++gCounter; 				// Increment counter

    // handle clock in 24-hour format
    gClock.seconds += 2;
    if (gClock.seconds > 59)   // Check for seconds -> minutes overflow
    {
      gClock.seconds = 0;

      gClock.minutes++;
      if (gClock.minutes > 59) // Check for minutes -> hours overflow
      {
        gClock.minutes = 0;

        gClock.hours++;
        if (gClock.hours > 23) // Check for hours -> day overflow
        {
          gClock.hours = 0;
        }
      }
    }
	}

  if (IOCIF == 1) // Interrupt-on-Change (used to wake from sleep)
  {
    if (IOCBF)  // Pushbutton/DIP switches
      IOCBF = 0;  // Clear all port B interrupt flags
    
    if (RC5_DECODER_IOCxF)  // RC5 Decoder
    {
        RC5_DECODER_interruptHandler(); // RC5 Decoder processing
        RC5_DECODER_IOCxF = 0;          // Clear IOC flag
    }
  }
}


/****************************************************************************
 ************************ Core Functions ************************************
 ***************************************************************************/

void main(void)
{
	hardware_init();	  // initialize hardware
  RC5_DECODER_init(); // initialize RC5 decoder

  while(1)
	{
    check_rc5();      // check for a properly decoded IR RC5 packet    
    check_buttons();  // check current state of buttons/DIP switches and set gSW accordingly
    change_mode();    // change system state, if necessary
    
    /*if (RC5_Decode.valid == TRUE) 
      RC5_Decode.valid = FALSE; // Clear flag*/
    
		/*switch (gMode)
		{
		 	case START_UP:
        if (gSW == SW_R)
          EN_MOTOR = HIGH;
        else
          EN_MOTOR = LOW;
  			break;	
		 	case SET_TIME:
  			break;
      case CLOCK_MODE:
        break;
		}*/

#ifndef __DEBUG
    SLEEP();  // sleep after every pass to minimize current use (will wake on the next interrupt)
#endif
	}			
}

/**************************************************************/

// From Microchip's IR RC5 Transmitter & Receiver demonstration code
// www.embeddedcodesource.com/codesnippet/ir-remote-control-philips-rc5-protocol-encoder-decoder/
void check_rc5(void) 
{
    byte count;
    uint16 stream;
    static uint16 RC5decode;

     // Check if new data is available
    if (RC5_DECODER_codeReady()) 
    {
        RC5decode = RC5_DECODER_getCode(); // Grab the Rx data bit stream
        RC5_Decode.toggle = RC5_DECODER_getToggleBit(RC5decode);
        RC5_Decode.address = RC5_DECODER_getDeviceAddr(RC5decode);
        RC5_Decode.command = RC5_DECODER_getCmd(RC5decode);
        
        NOP(); // *** SET BREAKPOINT HERE & monitor the RC5_Decode structure ***
    }
    
    RC5_DECODER_timeoutIncrement(); // update RC5 decoder timeout timer
}

/**************************************************************/

void check_buttons(void)
{
  /*if (!(SW_LEFT && SW_RIGHT))  // one or both of the buttons have been pressed
  {
    __delay_ms(50);  // debounce delay must be > than worst case switch bounce
    gSW = (!SW_LEFT << 1) | !SW_RIGHT;   // set global variable accordingly
  }
  else
    gSW = SW_NONE;*/
}

/**************************************************************/

void change_mode(void)
{
  /*switch (gMode)
  {
   	case START_UP:
   		if (gSW == SW_L)
   		{
   			gMode = SET_TIME;
   		}	  
      break;
	case SET_TIME:
   		if (gSW == SW_L)
   		{
   			gMode = START_UP;
   		}
      else if (gSW == SW_R)
      {
        gMode = CLOCK_MODE;
      }
    case CLOCK_MODE:
      if (gSW == SW_BOTH)
      {
   			gMode = SET_TIME;
      }
      break;
 	}*/
}

/**************************************************************/

void hardware_init(void) 	// Configure hardware to a known state
{
  // oscillator
  OSCCONbits.SPLLEN = 0;    // PLL disabled
  OSCCONbits.SCS = 0b10;    // System clock select: Internal oscillator
  OSCCONbits.IRCF = 0b1101; // 4MHz HF

  // internal pull-ups
  OPTION_REGbits.nWPUEN = 0; // enabled by individual WPUx latch values
  WPUA = 0x00;
  WPUB = 0x00;
  WPUC = 0x00;

  // set port input levels
  INLVLA = 0b00000100;  // RA2 Schmitt Trigger, all others TTL
  INLVLB = 0b11100000;  // RB7..5 Schmitt Trigger, all others TTL
  INLVLC = 0x00;        // All TTL

	// configure GPIO
	TRISB5 = INPUT; 	// SW_DIP1
  TRISB6 = INPUT;   // SW_DIP0
	TRISB7 = INPUT; 	// SW_TEST
  TRISC2 = INPUT;   // VBAT_IN
	TRISC0 = OUTPUT;	// EN_FPGA
	
	// set unused GPIO
  TRISA2 = OUTPUT;  // Configured later by RC5_DECODER_init()
  RA2 = LOW;
	TRISB4 = OUTPUT;
	RB4 = LOW;
  TRISC1 = OUTPUT;
	RC1 = LOW;
  TRISC3 = OUTPUT;	
	RC3 = LOW;
  TRISC4 = OUTPUT;	
	RC4 = LOW;
  TRISC5 = OUTPUT;
	RC5 = LOW;
  TRISC6 = OUTPUT;	
	RC6 = LOW;
  TRISC7 = OUTPUT;	
  RC7 = LOW;
  
	// set default pin states
	EN_FPGA = LOW;

  // disable unused peripherals
  CLKRCON = 0x00;   // reference clock
  WDTCON = 0x00;    // watchdog timer
  FVRCON = 0x00;    // fixed voltage reference
  DACCON0 = 0x00;   // DAC
  SRCON0 = 0x00;    // SR latch
  CM1CON0 = 0x00;   // comparator 1
  CM2CON0 = 0x00;   // comparator 2
  T2CON = 0x00;     // timer 2
  T4CON = 0x00;     // timer 4
  T6CON = 0x00;     // timer 6
  MDCON = 0x00;     // data signal modulator
  CCP1CON = 0x00;   // CCP1
  CCP2CON = 0x00;   // CCP2
  CCP3CON = 0x00;   // CCP3
  CCP4CON = 0x00;   // CCP4
  SSP1CON1 = 0x00;  // SSP1
  CPSCON0 = 0x00;   // capacitive sensing

	// timer 0/rtcc
  // @ 4MHz system clock, timer 0 will overflow every 32uS
  // used for time measurement of RC5 signal
  OPTION_REGbits.TMR0CS = 0;  // Clock source: Internal (Fosc / 4)
  OPTION_REGbits.TMR0SE = 0;  // Source edge select: Increment on low-to-high transition on T0CKI pin
  OPTION_REGbits.PS = 0b100;  // Prescaler: 32
  OPTION_REGbits.PSA = 0;     // Prescaler assigned to Timer 0
  TMR0IE = 0;

  // timer 1
	// @ 32.768kHz, timer 1 will increment every 30.5uS and overflow every 2 seconds for full 16-bit count
  T1CONbits.TMR1CS = 0b10;  // Clock source: Crystal oscillator on T1OSI/T1OSO
  T1CONbits.T1CKPS = 0b00;  // Prescale 1:1
  T1CONbits.T1OSCEN = 1;    // Enable dedicated Timer 1/LP oscillator circuit
  T1CONbits.nT1SYNC = 1;    // Do not synchronize external clock input
  T1GCON = 0x00;            // Disable gating
  timer1_on();              // Turn on timer 1
  TMR1H = TMR1H_LOAD;       // Set timer 1 overflow time
  TMR1L = TMR1L_LOAD;
  TMR1IE = 1;               // Enable interrupt

  // INT external interrupt
  OPTION_REGbits.INTEDG = 0;  // Interrupt on falling edge of INT
  INTE = 1;                   // Enable interrupt

  // interrupt-on-change
  IOCAP = 0x00;         // Port A: No interrupt on change
  IOCBP = 0x00;
  IOCBP = 0b11100000;   // Port B, Positive edge detection ([7..5])
  IOCBN = 0b11100000;   // Port B, Negative edge detection ([7..5])
  IOCIE = 1;            // Enable interrupt

	// adc
  ANSELA = 0x00;        // Port A: All digital I/O or special function
  ANSELB = 0x00;        // Port B: All digital I/O or special function
  ANSELC = 0b00000100;  // Port C: RC2/AN6 analog, all others digital I/O or special function

  ADCON0bits.ADON = 0;      // Start with ADC off
  ADCON0bits.CHS = 0b00110; // AN6
  ADCON1bits.ADPREF = 0b00; // Vref+ = VDD
  ADCON1bits.ADNREF = 0;    // Vref- = VSS
  ADCON1bits.ADFM = 1;      // Right justified
  ADCON1bits.ADCS = 0b000;  // Conversion clock: Internal (Fosc / 2), Tad = 1 / (4000000 / 2) = 0.5uS
  ADIE = 0;                 // Disable interrupt
		
	// enable interrupts
  PEIE = 1;   // Peripheral 
  GIE = 1;		// Global/system
}


/****************************************************************************
 ****************************** Timer/Clock *********************************
 ***************************************************************************/

void timer1_off(void)	// Disable Timer 1
{
  T1CONbits.TMR1ON = 0;
}

/**************************************************************/

void timer1_on(void)	// Enable Timer 1
{
  T1CONbits.TMR1ON = 1;

	while (!T1OSCR);      // wait for oscillator to start up and stabilize
}

/**********************************************************
 ************************ ADC *****************************
 **********************************************************/

uint8_t get_vbat(void)   // read battery voltage via ADC
{
  uint8_t   i;
	uint16_t  adc_value;		// ADC result (10-bit)
	//double    real_vbat;

  ADCON0bits.ADON = 1;    // turn on the ADC

  adc_value = 0;
	for (i = 0; i < 4; ++i)	// take multiple samples & average to reduce noise
	{
    ADCON0bits.GO_nDONE = 1;                // start conversion
    while (ADCON0bits.GO_nDONE);            // wait here until conversion is complete
    adc_value += ((ADRESH << 8) + ADRESL);  // store A/D value
	}
	adc_value >>= 2;      // calculate the average

  ADCON0bits.ADON = 0;  // turn ADC off to save power

  // set upper and lower bounds for A/D value
	/*if (adc_value > ADC_BOUND_UPPER) adc_value = ADC_BOUND_UPPER;
	else if (adc_value < ADC_BOUND_LOWER) adc_value = ADC_BOUND_LOWER;

  // convert ADC value to actual voltage
  real_vbat = 173.94 - (adc_value * 0.1889); // using slope-intercept equation from temperature look-up spreadsheet

  i = (uint8_t)real_vbat;   // cast to integer (discard the fractional component)
  return i;*/
  return adc_value;
}


/****************************************************************************
 ************************ Utility Functions *********************************
 ***************************************************************************/

