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

// Timeout trigger
uint8_t timeout_trigger;  // Number of minutes until trigger (FPGA enable)

// ADC
uint16_t adc_vbat;  // Current battery voltage: VBAT = (adc_vbat / 1023) * 3.0 * 1.4

// Flags (to communicate between functions and state change)
volatile input_state_type gSW;   // state of inputs (debounced)
uint8_t ir_trigger = FALSE;      // trigger received via IR (center button)
uint8_t disable_timer = FALSE;   // timer disabled via IR
uint8_t low_battery = FALSE;     // battery is below defined threshold


/****************************************************************************
 ********************** Interrupt Service Routines **************************
 ***************************************************************************/

void interrupt isr(void)
{
  static uint8_t tmr1_toggle = 0;
   
  if (TMR0IE == 1 && TMR0IF == 1)  // Timer 0: RC5 decoding (interrupt every 32.6ms)
  {
    TMR0IF = 0; // Clear the interrupt flag
  }
    
  if (TMR1IE == 1 && TMR1IF == 1)  // Timer 1: Clock/timing (interrupt every 1/2 seconds)
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
        nEN_FPGA = HIGH;  // C5G FPGA board off until triggered 
        break;
      case FPGA_ON:
        adc_vbat = get_adc();
        // Ensure battery is sufficiently charged to provide power to FPGA circuitry
        if (adc_vbat >= ADC_VBAT_MINIMUM) 
        {
          nEN_FPGA = LOW;   // Enable FPGA circuitry
          low_battery = FALSE;
        }
        else
        { 
          low_battery = TRUE;
        }
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
        switch (gSW.dipswitches) // Adjust the time offset depending on currently selected time duration
        {
          case SW_SML_TO:
            timeoffset = TIMEOUT_OFFSET_SML;
            break;
          case SW_MED_TO:
            timeoffset = TIMEOUT_OFFSET_MED;
            break;
          case SW_LGE_TO:
            timeoffset = TIMEOUT_OFFSET_LGE;
            break;
        }
        if (getAppleCommand() == APPLE_UP) // Increase trigger time
        {
          if ((timeout_trigger + timeoffset) < timeout_trigger)	// protect against wraparound
          {
            timeout_trigger = 0xFF;
          }
          else
          {
            timeout_trigger += timeoffset; // increase time like adding time to a microwave
          }
        }
        else // Decrease trigger time
        {
          if ((timeout_trigger - timeoffset) > timeout_trigger)	// protect against wraparound
          {
            timeout_trigger = 0x00;
          }
          else
          {
            timeout_trigger -= timeoffset; // decrease time (will trigger sooner)
          }
        }
        break;
      case APPLE_LEFT:    // Reset and re-enable the timer
        gClock.hours = gClock.minutes = gClock.seconds = 0;
        disable_timer = FALSE;
        break;
      case APPLE_RIGHT:   // Disable the timer
        disable_timer = TRUE;
        break;
      case APPLE_CENTER:  // Trigger FPGA on/off
        ir_trigger = TRUE;
        break;
      case APPLE_MENU:    // Reserved for future use
        break;
    }
    
    resetDecode(); // Reset the command after event has been handled (to avoid repeats)
  }
}

/**************************************************************/

void check_buttons(void)
{
  uint8_t dipSW = 0;
  
  if (SW_DIP1) dipSW |= 0b10;  // get state of DIP switches
  if (SW_DIP0) dipSW |= 0b01;
  if (dipSW != gSW.dipswitches)	// if DIP switches have changed while running, reset the trigger time
  {
    switch (dipSW)
    {
      case SW_NONE:   // Timer disabled
        timeout_trigger = 0;
        break;
      case SW_SML_TO: // Small trigger duration
        timeout_trigger = TIMEOUT_TRIGGER_SML;
        break;
      case SW_MED_TO: // Medium trigger duration
        timeout_trigger = TIMEOUT_TRIGGER_MED;
        break;
      case SW_LGE_TO: // Large trigger duration
        timeout_trigger = TIMEOUT_TRIGGER_LGE;
        break;
    }
  }
  gSW.dipswitches = (dipswitch_state_type)dipSW;

  //debounce the test pushbutton
  if (!SW_TEST && (gSW.button == FALSE))  // state change from high to low
  {
    __delay_ms(20);  // debounce delay must be > than worst case switch bounce
    if (!SW_TEST)
    {
      gSW.buttonevent = TRUE;
      gSW.button = TRUE;
    }
  }
  else if (SW_TEST && (gSW.button == TRUE)) // state change from low to high
  {
    __delay_ms(20);  // debounce delay must be > than worst case switch bounce
    if (SW_TEST)
    {
      gSW.buttonevent = TRUE;
      gSW.button = FALSE;
    }
  }
}

/**************************************************************/

void change_mode(void)
{
  switch (gMode)
  {
   	case START_UP:
      gClock.hours = gClock.minutes = gClock.seconds = 0;  // reset clock
      switch (gSW.dipswitches)
      {
        case SW_NONE:
          timeout_trigger = 0;
          break;
        case SW_SML_TO:
          timeout_trigger = TIMEOUT_TRIGGER_SML;
          break;
        case SW_MED_TO:
          timeout_trigger = TIMEOUT_TRIGGER_MED;
          break;
        case SW_LGE_TO:
          timeout_trigger = TIMEOUT_TRIGGER_LGE;
          break;
      }
   	  gMode = TIMER_WAIT;
      break;
	case TIMER_WAIT: // wait for a trigger event (timer, IR, test button)
      if ((gSW.buttonevent && gSW.button) || ir_trigger)
      {
        gMode = FPGA_ON;
        gSW.buttonevent = FALSE;
        ir_trigger = FALSE;
      }
      else if (!disable_timer) // if the timer is enabled
      {
        switch (gSW.dipswitches)
        {
          case SW_NONE:
            break;
          default:
            if (gClock.minutes >= timeout_trigger) // check time against remaining trigger duration
            {
              gMode = FPGA_ON;
            }
            break;
        }
      }
      break;
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
  TRISC0 = OUTPUT;	// nEN_FPGA 
	
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

  // set default pin states
	nEN_FPGA = HIGH;  // C5G FPGA board off until triggered
  
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
  ADCON0bits.CHS = 0b00110; // AN6
  ADCON1bits.ADPREF = 0b00; // Vref+ = VDD
  ADCON1bits.ADNREF = 0;    // Vref- = VSS
  ADCON1bits.ADFM = 1;      // Right justified
  ADCON1bits.ADCS = 0b001;  // Conversion clock: Internal (Fosc / 8), Tad = 2uS
  ADCON0bits.ADON = 1;      // Turn on ADC
  __delay_us(2);            // Acquisition delay
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

  adc_value = 0;
  for (i = 0; i < 4; ++i)	// take multiple samples & average to reduce noise
  {
    ADCON0bits.GO_nDONE = 1;                // start conversion
    while (ADCON0bits.GO_nDONE);            // wait here until conversion is complete
    adc_value += ((ADRESH << 8) + ADRESL);  // store A/D value
  }
  adc_value >>= 2;      // calculate the average

  return adc_value;
}
