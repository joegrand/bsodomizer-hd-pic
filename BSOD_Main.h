//
// BSODomizer HD
// Copyright (c) 2016 Grand Idea Studio, Inc.
//
// Filename: 	BSOD_Main.h
// Author: 	  Joe Grand [www.grandideastudio.com]
//
// Description: Header file for the BSODomizer HD Front End Subsystem
//
// Distributed under a Creative Commons Attribution 3.0 United States license
// http://creativecommons.org/licenses/by/3.0/us/ 
//

#ifndef __BSOD_MAIN_H__
#define __BSOD_MAIN_H__

#include <XC.h>
#include <stdint.h>

#define __DEBUG                 // #define for debug capability
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
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

#ifdef __DEBUG
  #pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#else
  #pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#endif

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)


/**************************************************************************
************************** Definitions ************************************
***************************************************************************/
// pin assignments
#define SW_DIP1     RB5		// Input
#define SW_DIP0     RB6   // Input
#define SW_TEST     RB7		// Input
#define EN_FPGA     RC0		// Output

// ADC
// adc_value = (Rt / (Rt + 10000)) * 1023
// Rt = ((adc_value / 1023) * 10000) / (1 - (adc_value / 1023))
/*#define ADC_BOUND_UPPER     750     // 32 F (0 C)
#define ADC_BOUND_LOWER     396     // 99 F (37.2 C)*/

// Timer 1
// load value = 65536 - (overflow_time / (1/32768))
#define TMR1H_LOAD  0x00    // 2 seconds
#define TMR1L_LOAD  0x00


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
  START_UP,
  SET_TIME,
  CLOCK_MODE
} mode_type;


typedef enum 	// state of the buttons
{
  SW_NONE = 0,
  SW_R    = 1,
  SW_L    = 2,
  SW_BOTH = 3
} button_state_type;


/***********************************************************************
 ************************** Function prototypes ************************
 ***********************************************************************/
 
void hardware_init(void);
void check_buttons(void);
void change_mode(void);
void check_rc5(void);

void timer1_off(void);
void timer1_on(void);

uint8_t get_vbat(void);

#endif /* __BSOD_MAIN__ */