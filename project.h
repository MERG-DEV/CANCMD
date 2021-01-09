#ifndef __PROJECT_H
#define __PROJECT_H

/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 CBUS Command Station - project definitions 

 This code is for a CBUS DCC Command Station

 This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                   and indicate if changes were made. You may do so in any reasonable manner,
                   but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                  your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                  legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

**************************************************************************************************************
  Note:   This source code has been written using a tab stop and indentation setting
          of 4 characters. To see everything lined up correctly, please set your
          IDE or text editor to the same settings.
******************************************************************************************************
 
*/ 

//
//  Pete Brownlow   31/3/11 Add status outputs on spare pins for scope debugging,
//                          introduce conditional compiler directive for BC1a
//                          and add definitions used by node parameters
//  Mike Bolton     19/4/11 Modified for 32 MHz clock
//  Pete Brownlow   27/6/11 Add CAN transmit error checking, beep twice if unable to transmit on CAN
//                          Output bridge enable turned off during overload
//  Pete Brownlow   25/2/12 Additional hardware types, re-org for FLiM libraries
//          For full project revision history see cancmd.c

#define MAJOR_VER 	4         // Major version number - add 100 for WIP
#define MINOR_VER 	'e'       // Minor version character
#define BETA            3     // Build number - set to 0 for release version
#define MODULE_FLAGS    PF_BOOT + PF_PRODUCER + PF_FLiM  // Producer, boot, FLiM
#define BUS_TYPE        PB_CAN
#define LOAD_ADDRESS    0x0800     // Need to put in parameter block at compile time, only known to linker so hard code here
#define MNAME_ADDRESS   LOAD_ADDRESS + 0x20 + sizeof(ParamBlock)   // Put module type string above params so checksum can be calculated at compile time

// Check hardware defined

#ifndef CANCMD
  #ifndef CANKCMD
    #ifndef BC1a
      #ifndef CANCSB
        #ifndef CANGC3
          #ifndef ANIMATED
            No hardware defined - set command line switch -D<hardwaretype> in project build options
          #endif          
        #endif
      #endif
    #endif
  #endif
#endif

// #include "devincs.h"

#include "cbus.h"

// CANCMD headers
#include "cancmd.h"
#include "cmdFLiM.h"
#include "commands.h"
#include "power_cmds.h"
#include "program_cmds.h"
#include "mode_cmds.h"
#include "packet_gen.h"


#include "romops.h"               
// #include "cbusxdefs.h"
#include "isr_high.h"


// Only CANCMD, CANBC and CANCSB fully implemented
// CANBC is the original MERG BC1a command station/booster fitted with a CANBC CBUS daughter board                    
// CANCSB is a derivation of CANCMD with additional 3 Amp booster on board                    
// CANGC3 is the Rocrail version of CANCMD, currently runs standard CANCMD firmware
// CANKCMD will be port for PIC18F25k80                     
// The "Animated conrtroller" was going to be a variation by Chuck Hoelzen for his "Animated" system - no news on that recently
                    
                    
#if CANCMD
  #define MANU_ID       MANU_MERG
  #define MODULE_ID 	MTYP_CANCMD
  #define MODULE_TYPE   "CANCMD"
  #define CPUID         P18F2580
#elif CANCSB
  #define MANU_ID       MANU_MERG
  #define MODULE_ID 	MTYP_CANCSB
  #define MODULE_TYPE   "CANCSB"
  #define CPUID         P18F2580
#elif CANKCMD
  #define MANU_ID       MANU_MERG
  #define MODULE_ID 	MTYP_CANCMD
  #define MODULE_TYPE   "CANCMD"
  #define CPUID         P18F25k80
#elif BC1a
  #define MANU_ID       MANU_MERG
  #define MODULE_ID	MTYP_CANBC
  #define MODULE_TYPE   "CANBC"
  #define CPUID           P18F4580
#elif CANGC3
  #define MANU_ID       MANU_ROCRAIL
  #define MODULE_ID     MTYP_CANGC3
  #define MODULE_TYPE   "CANGC3"
  #define CPUID         P18F2580
#elif ANIMATED
  #define MANU_ID       MANU_SPECTRUM
  #define MODULE_ID     MTYP_AMCTRLR
  #define MODULE_TYPE   "AMCTRLR"
  #define CPUID         P18F25k80
#endif


extern rom unsigned char defaultID;
extern rom unsigned char status;
extern rom unsigned short nodeID;


// DEFINE DEFAULT VALUES

#define RETRY_DELAY     9*256	// Retry after short =  9 * 256 * timer interrupt of 232us ~ 535ms
#define OVLD_DELAY	130	// Delay after overload detect before cutout = 130*232us = 30160us = 30ms
#define	I_ACK_DIFF	4	// A-D steps increment for 60mA ACK pulse
#define HEARTBEAT	500000	// mS for heartbeat interrupt
#define	DCC_BASE_PERIOD	58	// uS for DCC base period, from which the rest is derived
#define FLASH_INTERVAL  4       // No. of heartbeats between flashes of LED to show alive

#if CANCMD

    #define I_LIMIT_MAIN	768
    #define I_LIMIT_SVC		768

#elif CANCSB

    #define I_LIMIT_MAIN	768
    #define I_LIMIT_SVC		768


#elif BC1a

    #define I_LIMIT_MAIN	100
    #define I_LIMIT_SVC		768

#endif


// THESE DEFAULT VALUES ARE STORED IN NODE VARIABLES AND CAN BE OVERIDDEN BY FLiM

#define DEFAULT_CS_NUM          0           // To allow for multiple command stations in the future
#define	DEFAULT_WALK_TIMEOUT	60			// Seconds for walkabout timeout
#define DEFAULT_SOD_DELAY       1           // Seconds after track power on for SOD message
#define DEFAULT_HONK_INTERVAL   2           // Default honk/whistle every other time in POC shuttle
#define DEFAULT_MAX_SPEED       130         // Default is higher than maximum possible so no speed limiting

#if CANCMD

    #define	DEFAULT_USER_FLAGS	0b01110110  // Stop on timeout, permit share and steal, map events to DCC accessory commands, send Start event
    #define DEFAULT_OP_FLAGS    0b01000111  // J7 controls output, analogue current detect, main is onboard
    #define DEBUG_FLAGS         0x00        // Which CBUS packets to send with DCC packet data for debugging
    #define	MAIN_CURRENT_LIMIT	96			// Raw A->D value for 1A current limits
    #define	SVC_CURRENT_LIMIT	96
    #define	MAIN_CURRENT_MULT	10			// Multiplier to give milliamps
    #define AMPERAGE_REPORTING  0           // Default time in half seconds between amperage reporting messages - zero is disabled (can be changed in NV)
    #define	SVC_ACK_DIFF		3			// Increase in current for ack puluse to be detected
    #define	SHOOTTHRU_DELAY		0			// Not required in cancmd

#elif CANCSB

    #define	DEFAULT_USER_FLAGS	0b01110110  // Stop on timeout, permit share and steal, map events to DCC accessory commands, send start event
    #define DEFAULT_OP_FLAGS    0b01000110  // Main output is on board booster, analogue current detect
    #define DEBUG_FLAGS         0x00        // Which CBUS packets to send with DCC packet data for debugging
    #define	MAIN_CURRENT_LIMIT	50			// Raw A->D value for main track current limit. Multiply by MAIN_CURRENT_MULT to give the current limit in milliAmps. Default gives 3 Amps - reduce a bit if using a 3 Amp power supply
    #define	SVC_CURRENT_LIMIT	96          // Raw A->D value for 1A current limit on programming track
    #define	MAIN_CURRENT_MULT	50			// Multiplier to give milliamps
    #define AMPERAGE_REPORTING  .10         // Default time in half seconds between amperage reporting messages - zero is disabled (can be changed in NV)
    #define	SVC_ACK_DIFF		3			// Increase in current for ack puluse to be detected
    #define	SHOOTTHRU_DELAY		0			// Not required in cancmd or cancsb

#elif BC1a

    #define	DEFAULT_USER_FLAGS	0b11110110  // Stop on timeout, permit share and steal, map events to DCC accessory commands, send start event, shuttles on
    #define DEFAULT_OP_FLAGS    0b00000110  // Main output is on board booster, analogue current detect
    #define DEBUG_FLAGS         0x00        // Which CBUS packets to send with DCC packet data for debugging
    #define	MAIN_CURRENT_LIMIT	125			// Raw A->D value for 5A current limit
    #define	SVC_CURRENT_LIMIT	96			// Service track is same circuit as cancmd
    #define	MAIN_CURRENT_MULT	30			// Multiplier to give milliamps
    #define AMPERAGE_REPORTING  0           // Time in seconds between amperage reporting messages (0 to disable)
    #define	SVC_ACK_DIFF		5			// Increase in current for ack puluse to be detected
    #define	SHOOTTHRU_DELAY		8			// Protection for FETs

#endif



// DEFINE INPUT AND OUTPUT PINS

//
// Port A analogue inputs and digital I/O
//

#define ADCON2_INIT	0b10000110      // result right justified, Fosc/64, common to both hardware

#if BC1a
  // BC1a
  // RA0 is AN0 current sense for main track (if BC1a mod done)
  // RA1 is AN1 ack current sense for programming track (L6202)
  //

  #define ADCON1_INIT	0b00001101	// Internal Vref, AN0 and AN1 analogue input
  #define ADCON0_SVC	0b00000101	// RA1 for service track, A/D on
  #define ADCON0_MAIN	0b00000001	// RA0 for main track, A/D on

  #define OLOAD_DETECTN	PORTAbits.RA4	// Overload detect digital input on BC1a (active low)

  #define PORTA_DDR	0b11111111
  #define PORTA_INIT	0x0

#elif CANCMD
  // CANCMD
  // RA0 is AN0 current sense
  //
  #define ADCON1_INIT	0b00001110	// Internal Vref, AN0 analogue input
  #define ADCON0_SVC	0b00000001	// RA0 for service track, A/D on
  #define ADCON0_MAIN	0b00000001	// RA0 for main track, A/D on

  #define OLOAD_DETECTN	1		// No digital overload input on cancmd, so always inactive

  #define AWD         	LATAbits.LATA1	// Sounder

  #define SW          	PORTAbits.RA2	// Flim switch
  #define SPARE1        PORTAbits.RA3   //
                                        // RA4 pin 6 not used as pin is a capacitor on K series - initialise as an output
  #define SPARE2        PORTAbits.RA5   //
                                        // RA6-7 not available - used for oscillator
  #define PORTA_DDR	0b11000101	//
  #define PORTA_INIT	0x0

#elif CANCSB
  // CANCSB
  // Original hardware RB4 is AN9 main track current sense, but on 2480 has to be RA0/AN0
  // Original hardware RA3 is AN3 programming track current sense but on 2480 has to be RA1/AN1
  //
  #define ADCON1_INIT	0b00001101	// Internal Vref, AN0 and AN1 analogue input
  #define ADCON0_SVC	0b00000101	// RA1 for service track, A/D on
  #define ADCON0_MAIN	0b00000001	// RA0 for main track, A/D on

  #define OLOAD_DETECTN	1		// No digital overload input on cancsb, so always inactive

  #define SW          	PORTAbits.RA2	// Flim switch
  #define DCC_EN        PORTAbits.RA3   // Main track DCC enable
  #define ALARM         PORTAbits.RA5   // Alarm input from booster
                                        // RA4 pin 6 not used as pin is a capacitor on K series - initialise as an output
                                        // RA6-7 not available - used for oscillator
  #define PORTA_DDR	0b11100111	//
  #define PORTA_INIT	0x0
#endif

//
// Port B
// Note that RB2 and RB3 are canrx and cantx so not used as I/O pins - but DDR (TRISB) bit 3 must be set
//           RB6 and RB7 are used as PGC and PGD but can also drive LEDs
//


#if BC1a
  // BC1a port b was serial data from the old handsets, which are not used here

  #define SWAP_OP	0               // Main and service mode outputs fixed, no jumper required

  #define DCC_SVC_POS	PORTBbits.RB0	// Service track pos DCC drive
  #define DCC_SVC_NEG	PORTBbits.RB1	// Service track neg DCC drive
  #define DCC_SVC_EN	PORTBbits.RB4	// Service track output enable
                                        // RB2 and RB3 not available - used for CANTX and CANRX
#define LED1Y    	PORTBbits.RB6	// Yellow (canbc1a has headers for leds here, which may be fitted)
#define LED2G    	PORTBbits.RB7	// Green

  #define PORTB_DDR    0b00101100          // 
  #define PORTB_DDRTD  0b00101100      // Data direction register when train detector inputs for shuttle are enabled (not port B on BC1a)
  #define PORTB_DDRCO  0b00101100      // Data direction register when input for railcom cutout is enabled (not port B on BC1a)


  #define PORTB_INIT	0x00

#elif CANCMD
  #define cutoutIn  PORTBbits.RB0   // RB0 Pin 21  Spare input used as input to initiate cutout (cannot be used with ToTi input feature)
  #define TOTI1 	PORTBbits.RB0	// RB0 Pin 21  Spare inputs can be used for train detectors for simple shuttle (cannot be used with cutout input feature)
  #define TOTI2     PORTBbits.RB1	// RB1 Pin 22
                                        // RB2 and RB3 not available - used for CANTX and CANRX
  #define ALARM     PORTBbits.RB4   // RB4 Pin 25  Alarm input from booster
  #define SWAP_OP 	PORTBbits.RB5	// RB5 Pin 26  J7 Jumper for main track output on external booster or on board track output
  #define LED1Y    	PORTBbits.RB6	// Yellow (canbc1a has headers for leds here, which may be fitted)
  #define LED2G    	PORTBbits.RB7	// Green
  
  #define PORTB_DDR    0b00111100
  #define PORTB_DDRTD  0b00111111      // Data direction register when train detector inputs for shuttle are enabled 
  #define PORTB_DDRCO  0b00111101      // Data direction register when input for railcom cutout is enabled

  #define PORTB_INIT	0x00

#elif CANCSB
  #define SWAP_OP	0               // Main and service mode outputs fixed, no jumper required

  #define DCC_OUT   PORTBbits.RB0   // DCC signal to booster 
  #define SHUTDOWN  PORTBbits.RB1   // Active low shutdown output to booster (active high after inverting by MIC4426)  
  
  #define DCC_POS	PORTBbits.RB4	// one side of main track H-bridge o/p
  #define DCC_NEG	PORTBbits.RB5	// other side main track o/p

  #define LED1Y    	PORTBbits.RB6	// Yellow
  #define LED2G    	PORTBbits.RB7	// Green

  #define PORTB_DDR    0b00001100
  #define PORTB_DDRTD  0b00001100      // Data direction register when train detector inputs for shuttle are enabled (not port B on CANCSB)
  #define PORTB_DDRCO  0b00001100      // Data direction register when input for railcom cutout is enabled (not port B on CANCSB)

  #define PORTB_INIT	0x00
  #define DCC_PORT	PORTB
#endif

//
// Port C DCC and diagnostic outputs
//
#if BC1a
  #define DCC_PKT_PIN	PORTCbits.RC7	// (pin 26) For scope debug - Set during packet send (can sync scope on packet start)

  #define cutoutIn      PORTCbits.RC7   // Cutout input for Railcom cutout when input active (cannot be used with ToTi input feature)
  #define TOTI1         PORTCbits.RC7   // Train detector input for shuttle use (feature not tested on BC1a)
  #define TOTI2         PORTCbits.RC6   // Train detector input for shuttle use

  #define DCC_POS	PORTCbits.RC3	// one side of booster H-bridge o/p
  #define DCC_NEG	PORTCbits.RC2	// other side o/p
  #define OTHERFAULT 	PORTCbits.RC1	// Other fault output (turns on fault led and sounder)
  #define OVERFAULT	PORTCbits.RC0	// Overload output (turns on overlaod led and sounder)
  #define DCC_PORT	PORTC

  #define PORTC_DDR    0b00000000
  #define PORTC_DDRTD  0b11000000      // Data direction register when train detector inputs for shuttle are enabled (feature not tested on BC1a)
  #define PORTC_DDRCO  0b10000000      // Data direction register when input for Railcom cutout is enabled (feature not tested on BC1a)

  
  #define PORTC_INIT	0x0
#elif CANCMD
  #define OVERLOAD_PIN 	PORTCbits.RC7	// (pin 18) For scope debug - set bit when overload detected
  #define ISR_PIN       PORTCbits.RC6	// (pin 17) For scope debug - set bit when in high priority ISR
  #define DCC_PKT_PIN	PORTCbits.RC5	// (pin 16) For scope debug - Set during packet send (can sync scope on packet start)      (also SDI)
  #define SHUTDOWN      PORTCbits.RC4   // (pin 15) Active low shutdown output to booster (active high after inverting by MIC4426) (also SDO)
  #define DCC_OUT       PORTCbits.RC3   // DCC signal to booster                                                                    (also SCK)
  #define DCC_NEG	PORTCbits.RC2	// other side o/p
  #define DCC_POS	PORTCbits.RC1	// one side of on board track output H-bridge o/p
  #define DCC_EN        PORTCbits.RC0
  #define DCC_PORT	PORTC

  #define PORTC_DDR     0b00000000
  #define PORTC_DDRTD   0b00000000      // Data direction register when train detector inputs for shuttle are enabled (not port C on CANCMD)
  #define PORTC_DDRCO   0b00000000      // Data direction register when train detector input for railcom cutout is enabled (not port C on CANCMD)

  #define PORTC_INIT	0x00010000

#elif CANCSB
  #define DCC_PKT_PIN	PORTCbits.RC0	// (pin 11) For scope debug - Set during packet send (can sync scope on packet start)
  #define OVERLOAD_PIN  PORTCbits.RC1	// (pin 12) For scope debug - set bit when overload detected
  #define ISR_PIN       PORTCbits.RC4   // (pin 15) For debug, set bit when in ISR (cannot be used with ToTi input feature)
  
  #define TOTI1         PORTCbits.RC4   // Train detector input for shuttle use (cannot be used with ISR pin debug feature)
  #define TOTI2         PORTCbits.RC2   // Train detector input for shuttle use

  #define AWD         	LATCbits.LATC3	// Sounder

  #define DCC_SVC_POS	PORTCbits.RC7	// Service track pos DCC drive
  #define DCC_SVC_NEG	PORTCbits.RC6	// Service track neg DCC drive
  #define DCC_SVC_EN	PORTCbits.RC5	// Service track output enable

  #define PORTC_DDR     0b00000000      // Default data direction register
  #define PORTC_DDRTD   0b00010100      // Data direction register when train detector inputs for shuttle are enabled
  #define PORTC_DDRCO   0b00010000      // Data direction register when input for Railcom cutout is enabled

  #define PORTC_INIT	0x00100000
#endif

//
// Port D  (BC1a only)
//
// Note - pin 29, RD6 is connected to the clock drive output for the old
//				handsets, so should be set high impedance
//
#if BC1a
  #define OPTION_J2	PORTDbits.RD3	// Option jumper (not used yet)
  #define OPTION_J3	PORTDbits.RD2	// Option jumper (not used yet)

  #define OVERLOAD_PIN	PORTDbits.RD1	// Diagnostic output for when overload detected
  #define ISR_PIN	PORTDbits.RD0	// Diagnostic output for when in ISR

  #define PORTD_DDR	0b11111100
  #define PORTD_INIT	0x0
#endif

//
// Port E  (BC1a only)
//
#if BC1a
  #define SW          	PORTEbits.RE0	// FLiM switch

  #define PORTE_DDR	0b11111111
  #define PORTE_INIT	0x0
#endif



// 14 Clear Timer interrupt flag
// Reload TMR0 for interrupt every 58us
// Tcyc = 125ns @ 32MHz
// Interrupt every 58/(.125) = 464 Tcyc
// Value loaded into TMR0 needs to be adjusted for:
// - TMR0 interrupt latency (3 Tcyc)
// - Number of instructions from interrupt to reload
// - TMR0 inhibit after reload (2 Tcyc with 4:1 prescaler)
// Prescaler = 4:1
// So value is 0 - (464 - 3 - 30 - 2)/4 = -107 = 0x95

#define TMR0_NORMAL	0x95

// For Railcom, we need an interrupt for a half the usual period when we need to start the Railcom cutout 29uS after the first preamble bit start
// 29uS instead of 58uS
// Interrupt after 29/.125 = 232 Tcyc
// Value is 0 - (232 - 3 - 30 - 2)/4 = -49 = 0xCF

#define TMR0_RAILCOM  0xCF

	// for ZTC mode we modify the bit time to 52us
	// So value is 0 - (416 - 3 - 30 - 2)/4 = -95 = 0xA0
#define TMR0_ZTC	0xA1


#endif	// __PROJECT_H
