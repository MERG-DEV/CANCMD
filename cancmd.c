/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 CBUS Command Station - main processing loop

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

// Revision History
//
// 01/08/09     Created from sprogII_2_6
//			k	Modify LED state
//				Add overload retry for mini booster
//			l	 Add AWD output, modify LED behaviour
// 24/11/10	m	 Lower BOR voltage to 2.8V
//			n	 Support for throttle Em stop
// 23/12/10 p	 Initialize retry delay
// 06/01/11	r	 Set ave, sum, retry_delay & ovld_delay = 0 at power on
//				 during programming
//				 Wait for reset packets to complete before sampling ICCQ
//				 Remove redundant power_on/off_cycle code	
// 01/04/11	s	 Pete Brownlow -Added module paramters so can be bootloaded from FLiM config utility
//               Updated cbusdefs.h from latest CBUS specification
//               BC1a definition added ready for port to BC1a
//				Fixed loading CAN_ID so correct value loaded into TX buffer (spotted by Roger Healey)
//				Added diagnostic output signals on unused pins
// 06/04/11		Roger Healey	Initialise Node_id to DEFAULT_NN
//		`		Support for parameter read and firmware update from FLiM config utility
// 07/04/11		2a	Pete Brownlow	First release candidate for shipment with cancmd kits				
//						cbusdefs updated for release 7e of the CBUS spec (no effect on cancmd build)
// 16/4/11		2b	Pete Brownlow	Long preamble increased from 20 to 24 to ensure meeets NMRA spec and
//						to see if it will now program Tsunami decoders
// 19/04/11			Mike Bolton	Mod to 2a for 32 MHz clock  
// 19/04/11		3a	Pete Brownlow	Incorprate Mikes changes for 32MHz clock, keep slightly increased preamble of 22
//						Change to new major version no. to emphasise change in hardware required (8 MHz resonator)
// 21/04/11		3b	Mike Bolton	Bootloader modified to intialise CAN timing for 32MHz clock - source file now boot3.asm
// 26/06/11		3c	Pete Brownlow	Add CAN transmit error checking, timeout if send fails, diagnostic beeps if unable to send on CAN, 
//						turn off output bridge enable during a short
// 18/07/11		3d	Pete Brownlow	Add directives to library routines c018c and p18f2580.asm so they link out of bootloader area
//  2/01/12		3e	Pete Brownlow	Implement WCVOA  (also built with ver 3.4 of the Microchip C18 compiler)
//  22/3/12             3f      Pete Brownlow   Fixed bug in recovery_time which meant ack received flag was claered - affected write CV in service mode on some decoders inc Zimo and Loksound
//                                              Added MODULE_FLAGS parameter, reserved space up to 0840 for future parameters
//  2011-2017   4?      Pete Brownlow   Work in Progress for version 4 features - with steal/share, caching of loco speed/direction/function settings, FLiM with NVs for CANCVMD settings, POC shuttle
//  14/3/16     4CBeta12 Pete Brownlow   Increase main track preamble to 16 to allow for railcom
//                                       Version number to 4 from 104 now that FCU supports Beta flag and build no.
//  25/5/17     4dBeta1 Pete Brownlow   Add DCC accessory mapping from CBUS events - enabled by NV flag short/long and node number controlled by NV
//  02/06/17    4dBeta2 Pete Brownlow   Release candidate for version 4 - FLiM on by default, DCC accessory mapping on by default, POC shuttle controlled by NV flag, off by default
//                                      CANCSB support is work in progress - conditional compilation does not introduce any of the CANCSB changes unless the directive CANCSB is defined.
//  5/1/18      4dBeta5 Pete Brownlow   CANCSB support complete and tested.
// 21/5/18      4d      Pete Brownlow   Released as 4d - no changes except version number.
// 18/11/18     4eBeta1 Pete Brownlow   Add support for digital inputs from train detectors for shuttle
// 20/5/19      4eBeta2 Pete Brownlow   Add support for Railcom cutout during preamble
// 19/11/20     4eBeta3 Pete Brownlow   Complete debug and test of Railcom cutout support

// Definition of the version number is in project.h

#include "project.h"

#define BORSTAT BOHW

#pragma config OSC=HSPLL, FCMEN=OFF, IESO=OFF
// #pragma config PWRT=ON, BOREN=BOHW, BORV=2, WDT = OFF, WDTPS=256
#pragma config PWRT=ON, BOREN=BORSTAT, BORV=3, WDT = OFF, WDTPS=256
#pragma config MCLRE=ON, LPT1OSC=OFF, PBADEN=OFF, DEBUG=OFF
#pragma config XINST=OFF, BBSIZ=1024, LVP=OFF, STVREN=OFF
#pragma config CP0=OFF, CP1=OFF, CPB=OFF, CPD=OFF
#pragma config WRT0=OFF, WRT1=OFF, WRTB=OFF, WRTC=OFF, WRTD=OFF
#pragma config EBTR0=OFF, EBTR1=OFF, EBTRB=OFF


#pragma romdata EEPROM
// Define data stored in EEPROM
// Initialisation here ensures newly programmed part is in virgin state with defaults set

rom WORD EEcanID  = DEFAULT_CAN_ID;
rom WORD EEnodeID = DEFAULT_NN;
rom BOOL EEFlimMode = TRUE;

#pragma udata access VARS

// Flags for operations and DCC

near DccFlags   dcc_flags;
near OpFlags    op_flags;

// dcc packet buffers for service mode programming track
// and main track

near BYTE dcc_buff_s[7];
near BYTE dcc_buff_m[7];

#pragma udata MAIN_VARS

ModeWord mode_word;                 // Mode flags
BYTE    FlashInterval;              // Number of heartbeats between flashes of green LED to show alive
BOOL    Doubleflash;                // Control double flashing for FLiM
BYTE    FlashTime;                  // Number of main loops to hold flash of green LED on
BYTE	tmr0_reload;                // Reload value for DCC tming
BYTE    startupTimer;               // heartbeats for startup sequence
BYTE    startupEventTimer;            // heartbeats for delay in startup event


// local function prototypes
void setup(void);
void __init(void);

/*
 * Interrupt vectors
 */

#pragma code high_vector=0x808
void high_irq_errata_fix(void);
void HIGH_INT_VECT(void)
{
    _asm
        CALL high_irq_errata_fix, 1
    _endasm
}

/*
 * See 18F2480 errata
 */
void high_irq_errata_fix(void) {
    _asm
        POP
        GOTO isr_high
    _endasm
}


#pragma code low_vector=0x818
void LOW_INT_VECT(void)
{
    _asm GOTO isr_low _endasm
}

#pragma code APP

void main(void) {
    BYTE i;
  
    setup();    

    // Loop forever

    while (TRUE)
    {

        if (dcc_flags.dcc_overload)
        {
            // Programming overload
            dcc_flags.dcc_overload = 0;
            if (dcc_flags.dcc_test_outputs)
                OVERLOAD_PIN = 0;
        }

        if (swap_op == 0)
        {
            // On board booster
            if (dcc_flags.dcc_retry)
            {
                // Turn power back on after retry
                dcc_flags.dcc_retry = 0;
                op_flags.op_pwr_m = 1;
                if (dcc_flags.dcc_test_outputs)
                  OVERLOAD_PIN = 0;
            }
        }


        if (((dcc_flags.dcc_reading) || (dcc_flags.dcc_writing))
            && (dcc_flags.dcc_rdy_s == 1))
        {
            // iterate service mode state machine
            cv_sm((ModNVPtr) NVPtr);
        }

        if (dcc_flags.dcc_rdy_m)
        {
            // Main track output is ready for next packet
            packet_gen((ModNVPtr) NVPtr);
        }

        // Handle slot & service mode timeout every half second.
        // Also used for FLiM switch, beep delays and delayed events

        if (op_flags.slot_timer)
        {
            op_flags.slot_timer = 0;

            if (startupTimer != 0)
            {
                if (--startupTimer == 1)
                {
                    op_flags.op_pwr_m = TRUE;   // turn on track power
                    for (i = 0; i < 5; i++)
                        sendCbusOpc(OPC_ARST); //  // Send reset packets to reset cabs

                }
                if (startupTimer == 0)
                {
                    broadcast_stop();                   // Send stop all broadcast onto track (in case some decoders remember they were moving when power was removed)
                    CSStatus.EMStopAllActive = TRUE;
                    CSStatus.busOn = TRUE;
                    send_stat();                             // send command station status message (after additional half sec to ensure cabs had time to reset)
                    startShuttles(FALSE);
                    CSStatus.resetDone = FALSE;
                }    
            }
            else
                if (startupEventTimer != 0)
                    if (--startupEventTimer == 0)
                        sendStartOfDay( (ModEVPtr) NVPtr );     // If start of day event configured, send it now

            if (--FlashInterval == 0 )
            {
                LED2G = BlinkLED( 1 );	// Blink green LED on briefly periodically to show alive (twice for FLiM mode)
                // LED2G = 1;
                Doubleflash = !Doubleflash;
                FlashInterval = (((flimState != fsSLiM) && Doubleflash) ? 1 : FLASH_INTERVAL );
                FlashTime = 255;
            }
            
            checkToTiInputs();  // For shuttle operation
            FLiMSWCheck();
            check_session_timeouts( (ModNVPtr) NVPtr );
            amperage_messages();             

            if (BeepCount > 0)
            {
                op_flags.beeping = !op_flags.beeping;
                if (op_flags.beeping)
                {
                        BeepCount--;
                }
            }
            else {
                op_flags.beeping = FALSE;
            }

            setbeep((ModNVPtr) NVPtr);
           
            processDelayedEvents();

        }	// slot timer flag set

        // Check for Rx packet and setup pointer to it
        if ( isCanMsgReceived() ) {
            LED2G = BlinkLED( 1 );	// Blink green LED on whilst processing commands - to give indication how busy CS is
            // Decode the new command
            parse_cmd();
        }
        else
        {
            if (FlashTime != 0)
                FlashTime--;

            if (FlashTime == 0)
                LED2G = BlinkLED( 0 );
                // LED2G = 0;
        }
    } // main loop
} // main
 

void setup(void) {
    unsigned char i;

    // INitialise CS Status bits
    CSStatus.byte = 0;  // All flags false
    CSStatus.resetDone = TRUE;
        
    INTCON = 0;
    EECON1 = 0;

	//
    // setup initial values before enabling ports
    // Port A are analogue
    //
    cmdFlimInit();

    op_flags.byte = 0;	// Set all op flags to false
    BeepCount = 0;
    FlashInterval = FLASH_INTERVAL;
    Doubleflash = FALSE;
    FlashTime = 0;
 
    // Setup ports
    LATC = PORTC_INIT;
    LATB = PORTB_INIT;
    LATA = PORTA_INIT;

    if (cmdNVptr->opflags.enableCutoutInput)
    {    
        TRISC = PORTC_DDRCO; // Enable cutout input
        TRISB = PORTB_DDRCO;
    }    
    else if (cmdNVptr->opflags.enableTotiInputs)
    {    
        TRISC = PORTC_DDRTD; // Enable train detector inputs
        TRISB = PORTB_DDRTD;
    }    
    else
    {    
        TRISC = PORTC_DDR;
        TRISB = PORTB_DDR;
    }
    
    TRISA = PORTA_DDR;

#if BC1a

    PORTD = PORTD_INIT;
	PORTE = PORTE_INIT;
	
	TRISD = PORTD_DDR;
	TRISE = PORTE_DDR;

	DCC_POS = 0;		// Outputs off at startup to avoid any risk of shoot-through at power on
	DCC_NEG = 0;

	DCC_SVC_POS = 0;
	DCC_SVC_NEG = 0;
	DCC_SVC_EN = 0; 

	// sthru_delay = shoot_thru_nv;  // Load from program rom into ram for fast access during ISR
    
#elif CANCMD

        DCC_EN = 0;

#endif

    IPR3 = 0;			// All IRQs low priority for now
    IPR2 = 0;
    IPR1 = 0;
    PIE3 = 0b00100001;		// CAN TX error and FIFOWM interrupts
    PIE2 = 0;
    PIE1 = 0;
    INTCON3 = 0;
    INTCON2 = 0;                // Port B pullups are enabled
    INTCON = 0;
    PIR3 = 0;
    PIR2 = 0;
    PIR1 = 0;
    RCONbits.IPEN = 1;		// enable interrupt priority levels

    bit_flag_s = 6;			    // idle state
    bit_flag_m = 6;			    // idle state
    dcc_flags.word = 0;
    dcc_flags.dcc_rdy_m = 1;
    dcc_flags.dcc_rdy_s = 1;

    // Clear the refresh queue
    for (i = 0; i < MAX_HANDLES; i++) {
        q_queue[i].status.byte = 0;
        q_queue[i].address.addr_int = 0;
        q_queue[i].speed = 0x80;
        q_queue[i].fn1.byte = 0;
        q_queue[i].fn2.byte = 0;
        q_queue[i].fn2a.byte = 0;
        q_queue[i].timeout = 0;
    }
    q_idx = 0;
    q_state = 0;

    // Clear the send queue
    for (i = 0; i < 16; i++) {
        s_queue[i].status.byte = 0;
        s_queue[i].d[0] = 0;
        s_queue[i].d[1] = 0;
        s_queue[i].d[2] = 0;
        s_queue[i].d[3] = 0;
        s_queue[i].d[4] = 0;
        s_queue[i].d[5] = 0;
        s_queue[i].repeat = 0;
    }
    s_head = 0;
    s_tail = 0;

    mode_word.byte = 0;


    // Bootloader interlock bit
//    if (mode_word.boot_en == 1) {
//        mode_word.boot_en = 0;
//        cmd_wmode();
//    }

   
    // Programmer state machine
    prog_state = CV_IDLE;
    initCbus(EEcanID, EEnodeID );
    init_isr_high((ModNVPtr) NVPtr );

    // LED on, we are now running

    LED1Y = 0;
    LED2G = 1;

   // enable interrupts
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    startupTimer = 5;                           // 0.5s heartbeats - 2 secs to track on and send CBUS reset packets, another half second to send stat and start of day
    startupEventTimer = cmdNVptr->sodDelay + 1;   // Extra 0.5s beats before startup event sent (+1 for predecrement during initial test so it is sent once and only once)
}




// C intialiasation - declare a copy here so the library version is not used as it may link down in bootloader area

void __init(void)

{
}
