//
// BSODomizer HD
//
// Filename: 	BSOD_Main.c
// Author: 	  Joe Grand [www.grandideastudio.com] and Zoz
//
// Description: Main file for the BSODomizer HD Front End Subsystem
// Processor: 	Microchip PIC16LF1829
// Compiler: 	  MPLAB X IDE v3.35 w/ Microchip XC8 v1.38 (Free mode)
//
// Distributed under a Creative Commons Attribution 3.0 United States license
// http://creativecommons.org/licenses/by/3.0/us/ 
//

#include <XC.h>
#include <stdint.h>
#include "BSOD_Main.h"
#include "IR_Decoder.h"


/****************************************************************************
 ************************** Global variables ********************************
 ***************************************************************************/
 
volatile mode_type gMode = START_UP;

// Clock/Timer
struct time_struct
{
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} gClock = {0, 0, 0};  // Current time

// ADC
uint16_t adc_vbat;  // Current battery voltage: VBAT = (adc_vbat / 1023) * 3.0 * 1.4

// Flags
volatile input_state_type gSW;   // state of inputs (debounced)
uint8_t ir_trigger = FALSE;
uint8_t disable_timer = FALSE;
uint8_t low_battery = FALSE;


/****************************************************************************
 ********************** Interrupt Service Routines **************************
 ***************************************************************************/

void interrupt isr(void)
{
  static uint8_t tmr1_toggle = 0;
  
  if (INTF == 1)    // External INT
  {
    INTF = 0;
  }
  
  if (TMR0IE == 1 && TMR0IF == 1)  // Timer 0: RC5 decoding (interrupt every 32.6ms)
  {
    TMR0IF = 0; // Clear the interrupt flag
  }
    
  if (TMR1IE == 1 && TMR1IF == 1)  // Timer 1: Accurate clock/timing (interrupt every 1/2 seconds)
	{
    TMR1H = TMR1H_LOAD; // Reload Timer 1 overflow time
    TMR1L = TMR1L_LOAD;
		TMR1IF = 0;         // Clear the interrupt flag
    tmr1_toggle = !tmr1_toggle; // Toggle this value every interrupt
    
    // handle clock in 24-hour format
    if (!tmr1_toggle) // Every other interrupt, increment the seconds
      gClock.seconds += 1;
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

  if (IOCIE == 1 && IOCIF == 1) // Interrupt-on-Change
  {
    if (NEC_DECODER_IOCxF)  // IR Decoder
    {
      NEC_DECODER_interruptHandler(); // IR Decoder processing
      NEC_DECODER_IOCxF = 0;          // Clear IOC flag
    }
  }
}


/****************************************************************************
 ************************ Core Functions ************************************
 ***************************************************************************/

void main(void)
{ 
  hardware_init();	  // initialize hardware
  NEC_DECODER_init(); // initialize NEC/Apple Remote IR decoder

  while(1)
  {   
    check_buttons();  // check current state of buttons/DIP switches and set gSW accordingly
    check_ir();       // check for a properly decoded packet from the Apple Remote
    change_mode();    // change system state, if necessary
    
    switch (gMode)
    {
      case START_UP:
        break;
      case TIMER_WAIT:
        TRISC0 = INPUT;	  // nEN_FPGA 
        break;
      case FPGA_ON:
        /*adc_vbat = get_adc();
        if (adc_vbat >= ADC_VBAT_MINIMUM) // Ensure battery is sufficiently charged to provide power to FPGA circuitry
        {*/
          TRISC0 = OUTPUT;	// nEN_FPGA
          nEN_FPGA = LOW;   // Enable FPGA circuitry
          low_battery = FALSE;
        /*}
        else
        { 
          low_battery = TRUE;
        }*/
        break;
    }
	}			
}

/**************************************************************/

void check_ir(void) 
{
  uint8_t timeoffset = 0;
  
  // Check if data is available
  if (hasValidDecode() == TRUE) 
  {        
    switch (getAppleCommand())
    {
      case APPLE_UP:
      case APPLE_DOWN:
        switch (gSW.dipswitches)
        {
          case SW_5MIN:
            timeoffset = 1;
            break;
          case SW_10MIN:
            timeoffset = 2;
            break;
          case SW_30MIN:
            timeoffset = 5;
            break;
        }
        if (getAppleCommand() == APPLE_UP)
        {
          gClock.minutes += timeoffset;
        }
        else
        {
          gClock.minutes -= timeoffset;
        }
        break;
      case APPLE_LEFT:
        gClock.hours = gClock.minutes = gClock.seconds = 0;
        disable_timer = FALSE;
        break;
      case APPLE_RIGHT:
        disable_timer = TRUE;
        break;
      case APPLE_CENTER:
        ir_trigger = TRUE;
        break;
      case APPLE_MENU:
        break;
    }
    
    resetDecode();
  }
}

/**************************************************************/

void check_buttons(void)
{
  uint8_t dipSW = 0;
  
  if (SW_DIP1) dipSW |= 0b10;
  if (SW_DIP0) dipSW |= 0b01;
  gSW.dipswitches = (dipswitch_state_type)dipSW;

  //debounce the test pushbutton
  if (!SW_TEST)
  {
    __delay_ms(50);  // debounce delay must be > than worst case switch bounce
    if (gSW.button == SW_TEST)
    {
      gSW.buttonevent = TRUE;
    }
    gSW.button = !SW_TEST;
  }
  else
  {
    if (gSW.button != SW_TEST)
    {
      gSW.buttonevent = TRUE;
    }
  	gSW.button = FALSE;
  }
}

/**************************************************************/

void change_mode(void)
{
  switch (gMode)
  {
   	case START_UP:
      gClock.hours = gClock.minutes = gClock.seconds = 0;
   	  gMode = TIMER_WAIT;
      break;
	case TIMER_WAIT: // wait for a trigger event (timer, IR, test button)
      if ((gSW.buttonevent && gSW.button) || ir_trigger)
      {
        gMode = FPGA_ON;
        gSW.buttonevent = FALSE;
        ir_trigger = FALSE;
      }
      else if (!disable_timer)
      {
        switch (gSW.dipswitches)
        {
          case SW_NONE:
            break;
          case SW_5MIN:
            if (gClock.minutes >= 5)
            {
              gMode = FPGA_ON;
            }
            break;
          case SW_10MIN:
            if (gClock.minutes >= 10)
            {
              gMode = FPGA_ON;
            }
            break;
          case SW_30MIN:
            if (gClock.minutes >= 30)
            {
              gMode = FPGA_ON;
            }
            break;
        }
      }
    case FPGA_ON:
      if ((gSW.buttonevent && gSW.button) || ir_trigger || low_battery)
      {
        gMode = START_UP;
        gSW.buttonevent = FALSE;
        ir_trigger = FALSE;
      }
      break;
  }
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
  TRISC0 = INPUT;	  // nEN_FPGA 
	
  // set unused GPIO
  TRISA2 = OUTPUT;  
  RA2 = LOW;
  TRISB4 = OUTPUT;  // Configured later by NEC_DECODER_init()
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
  SSP2CON1 = 0x00;  // SSP2
  CPSCON0 = 0x00;   // capacitive sensing

	// timer 0/rtcc
  // @ 4MHz system clock, timer 0 will increment every 128uS and overflow every 32.6ms for full 8-bit count
  // used for time measurement of RC5 signal
  OPTION_REGbits.TMR0CS = 0;  // Clock source: Internal (Fosc / 4)
  OPTION_REGbits.TMR0SE = 0;  // Source edge select: Increment on low-to-high transition on T0CKI pin
  OPTION_REGbits.PSA = 0;     // Prescaler assigned to Timer 0
  OPTION_REGbits.PS = 0b110;  // Prescaler: 128
  TMR0IE = 1;                 // Enable interrupt

  // timer 1
	// @ 125kHz, timer 1 will increment every 8uS
  T1CONbits.TMR1CS = 0b00;  // Clock source: Internal instruction clock (Fosc/4)
  T1CONbits.T1CKPS = 0b11;  // Prescale 1:8
  T1CONbits.T1OSCEN = 0;    // Dedicated Timer 1 oscillator circuit disabled
  T1CONbits.nT1SYNC = 0;    // Synchronize clock input
  T1GCON = 0x00;            // Disable gating
  T1CONbits.TMR1ON = 1;     // Turn on Timer 1
	while (!T1OSCR);          // Wait for oscillator to start up and stabilize
  TMR1H = TMR1H_LOAD;       // Set Timer 1 overflow time
  TMR1L = TMR1L_LOAD;
  TMR1IE = 1;               // Enable interrupt

  // INT external interrupt
  OPTION_REGbits.INTEDG = 0;  // Interrupt on falling edge of INT
  INTE = 0;                   // Disable interrupt

  // interrupt-on-change
  IOCAP = 0x00;         // Port A: No interrupt on change
  IOCAN = 0x00;
  IOCBP = 0x00;         // Port B: No interrupt on change
  IOCBN = 0x00;
  IOCIE = 0;            // Disable interrupt

  // adc
  ANSELA = 0x00;        // Port A: All digital I/O or special function
  ANSELB = 0x00;        // Port B: All digital I/O or special function
  ANSELC = 0b00000100;  // Port C: RC2/AN6 analog, all others digital I/O or special function

  ADCON0bits.ADON = 0;      // Start with ADC off
  ADCON0bits.CHS = 0b00110; // AN6
  ADCON1bits.ADPREF = 0b00; // Vref+ = VDD
  ADCON1bits.ADNREF = 0;    // Vref- = VSS
  ADCON1bits.ADFM = 1;      // Right justified
  ADCON1bits.ADCS = 0b001;  // Conversion clock: Internal (Fosc / 8), Tad = 2uS
  ADIE = 0;                 // Disable interrupt
		
	// enable interrupts
  PEIE = 1;   // Peripheral 
  GIE = 1;		// Global/system
}


/**********************************************************
 ************************ ADC *****************************
 **********************************************************/

uint16_t get_adc(void)   // read voltage on ADC input
{
  uint8_t   i;
  uint16_t  adc_value;		// ADC result (10-bit)

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

  return adc_value;
}
