//
// BSODomizer HD
//
// Filename: 	BSOD_Main.h
// Author: 	  Joe Grand [www.grandideastudio.com] and Zoz
//
// Description: Header file for the BSODomizer HD Front End Subsystem
//
// Distributed under a Creative Commons Attribution 3.0 United States license
// http://creativecommons.org/licenses/by/3.0/us/ 
//

#include <stdint.h>

#ifndef __BSOD_MAIN_H__
#define __BSOD_MAIN_H__

//#define __BSOD_DEBUG            // #define for debug capability
#define _XTAL_FREQ  4000000     // oscillator frequency for __delay_xx() macros

/**************************************************************************
************************** Configuration Bits *****************************
***************************************************************************/

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

#ifdef __BSOD_DEBUG
  #pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#else
  #pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#endif

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected, 2.7V)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)


/**************************************************************************
************************** Definitions ************************************
***************************************************************************/
// pin assignments
#define SW_DIP1     RB5		// Input
#define SW_DIP0     RB6   // Input
#define SW_TEST     RB7		// Input
#define nEN_FPGA    RC0		// Output

// ADC
// adc_value = (VBAT / 1.4) / 3.0 * 1023
// LTC3605 Step-Down Regulators on C5G require minimum 4V input (but work down to 3.4V)
#define ADC_VBAT_MINIMUM    876 // 3.6V

// Timer 1
// load value = 65536 - (overflow_time / (1/125000))
#define TMR1H_LOAD  0x0B    // 1/2 second
#define TMR1L_LOAD  0xDC

// Timeout trigger & offset default values (in minutes)
#define TIMEOUT_TRIGGER_SML		5
#define TIMEOUT_TRIGGER_MED		10
#define TIMEOUT_TRIGGER_LGE		30
#define TIMEOUT_OFFSET_SML		1
#define TIMEOUT_OFFSET_MED		2
#define TIMEOUT_OFFSET_LGE		5

#define LOW         0
#define HIGH        1

#define OUTPUT      0
#define INPUT       1

#define	FALSE       0
#define TRUE        1


/**************************************************************************
************************** Macros *****************************************
***************************************************************************/

#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))
#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000.0)))


/**************************************************************************
************************** Typedefs ***************************************
***************************************************************************/

typedef enum	// operating mode
{
  START_UP,     // Start up
  TIMER_WAIT,   // Wait for trigger
  FPGA_ON       // BSODomy in action
} mode_type;

typedef enum 	// dip switch values
{
  SW_NONE  	= 0b00,
  SW_SML_TO = 0b01,
  SW_MED_TO = 0b10,
  SW_LGE_TO = 0b11
} dipswitch_state_type;

typedef struct	// state of the buttons/switches
{
	dipswitch_state_type dipswitches;
	uint8_t button;
	uint8_t buttonevent;
} input_state_type;


/***********************************************************************
 ************************** Function prototypes ************************
 ***********************************************************************/
 
void hardware_init(void);
void check_buttons(void);
void change_mode(void);
void check_ir(void);
uint16_t get_adc(void);

#endif /* __BSOD_MAIN_H__ */