/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 MERG CBUS DCC Command Station/Programmer - high priority Interrupt service routine

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
	
 For version number and full revision history see CANCMD.c
 
*/ 

//
//   	Mod by Mike B for PORTC,6 up at start of ISR and down at end.
//   	Pete Brownlow 	May 11  - Change A->D routine to make it generic for main/service track when one or both present
//      Pete Brownlow	27/6/11 - Change heartbeat to half second so can use for beep control and transmit timeouts as well
//              		  Output bridge enable turned off during overload retry wait and when
//				  service programming track is turned off
//      Pete Brownlow   19/11/20 - Ver 4e Beta - add support for Railcom cutout, enabled by NV flag
//	    Simon West	    24/02/24 - Added definition CANCMDB adding in shootthru for main track config for use with L298
//                                 and RCUT to have a high signal to drive logic MOSFETs during Railcom cutout.

// The high priority interrupt is triggered by a timer to generate the DCC bit stream and do A to D for current monitoring

#include "project.h"

//
// DCC bit state machine macros
//
#define FIRST_EDGE 0
#define SECOND_EDGE_BIT1 1
#define SECOND_MIDPOINT_BIT0 2
#define SECOND_EDGE_BIT0 3
#define FIRST_MIDPOINT_BIT0 4
#define SECOND_IDLE 5
#define FIRST_IDLE 6
#define SECOND_PRE 7
#define FIRST_PRE 8
#define SECOND_MID_START 9
#define SECOND_START 10
#define FIRST_MID_START 11
#define FIRST_START 12

//
// ADC state machine macros
//
#define AD_RESET 0
#define ACQ_MAIN 1
#define CONV_MAIN 2
#define ACQ_SVC 3
#define CONV_SVC 4


#pragma udata access VARS

near BOOL   swap_op;

near WORD main_retry_delay;
near WORD main_ovld_delay;
near WORD svc_ovld_delay;
near WORD imaxmain;
near WORD imaxsvc;
near WORD ilimitmain;
near WORD ilimitsvc;
near WORD iackdiff;
near WORD anmain;
near WORD avemain;
near WORD summain;
near WORD ansvc;
near WORD avesvc;
near WORD sumsvc;

near BYTE iccq;
near BYTE bit_flag_s;
near BYTE bit_flag_m;
near BYTE next_byte_s;
near BYTE pre_cnt_s;
near BYTE byte_cnt_s;
near BYTE next_byte_m;
near BYTE pre_cnt_m;
near BYTE byte_cnt_m;

near BYTE dcc_bits_s;
near BYTE dcc_bits_m;
near BYTE dcc_bytes_s;
near BYTE dcc_bytes_m;

near BYTE dcc_idx_s;
near BYTE dcc_idx_m;

near RailcomFlags railcomFlags;

// BYTE awd_count;

near BYTE sthru_delay;		// Shoot through delay initialised from NV
near BYTE sthru_count;		// Shoot-through delay counter

#pragma udata MAIN_VARS

BYTE                ad_state;
BOOL                analogue_main;	// Analogue input for main track
unsigned short long slot_timer;


//
// toggle_dcc()
//
// Called from ISR to toggle the internal DCC value.
//
#define toggle_dcc_s() op_flags.op_bit_s=!op_flags.op_bit_s
#define toggle_dcc_m() op_flags.op_bit_m=!op_flags.op_bit_m

#pragma code APP

// Initialisation routine for ISR variables

void init_isr_high( ModNVPtr NVPtr )
{
    swap_op = set_output( NVPtr );
    main_retry_delay = 0;
    main_ovld_delay = 0;
    svc_ovld_delay = 0;
    anmain = 0;
    avemain = 0;
    summain = 0;
    ansvc = 0;
    avesvc = 0;
    sumsvc = 0;
    analogue_main = NVPtr->opflags.mainancurrent;
    sthru_delay = NVPtr->shootthrudelay;
    imaxmain = NVPtr->maincurrentlimit;
    imaxsvc  = NVPtr->svccurrentlimit;
    iackdiff = NVPtr->ackdiff;
    ilimitmain = I_LIMIT_MAIN;
    ilimitsvc = I_LIMIT_SVC;
    iccq = 0;

    // Setup A/D - values depend on hardware are defined in project.h
    ADCON2 = ADCON2_INIT;		// result right justified, Fosc/64
    ADCON1 = ADCON1_INIT;	    // Internal Vref, enable analogue inputs that are used 
    ADCON0 = ADCON0_MAIN;	    // Channel for main track set initially. A/D on

    ad_state = 0;
	// Start heartbeat timer ( every half second )
    slot_timer = ((short long)HEARTBEAT)/DCC_BASE_PERIOD;

    tmr0_reload = TMR0_NORMAL;
    // set 52us period if ZTC mode
    if (mode_word.ztc_mode) tmr0_reload = TMR0_ZTC;

   // Set up TMR0 for DCC bit timing with 58us period prescaler 4:1,
   // 8 bit mode
   // T0CON = 0x40;  old value from 16MHz clock
    T0CON = 0x41;   // Timer 0 for 58uS with 32MHz clock
    TMR0L = 0;
    TMR0H = 0;
    INTCONbits.TMR0IE = 1;
    INTCON2bits.TMR0IP = 1;
    T0CONbits.TMR0ON = 1;
    
    railcomFlags.byte = 0;  // Clear all Railcom flags
    railcomFlags.railcomEnabled = NVPtr->opflags.enableRailcomCutout;
}



//
// Set beep state according to overload status, beeping flags and the alarm input from the booster
// Called from ISR and from main cancmd loop which manages error beeping counts
//

void setbeep( ModNVPtr NVPtr )

{
#if BC1a
    // AWD and overload LED drive signal

    OVERFAULT =  (main_retry_delay > 0); 
    OTHERFAULT = (!NVPtr->userflags.silent && ((svc_ovld_delay > 0) || op_flags.beeping));   //  ???? step through to check correct code generated

#elif CANCMD
    // AWD drive signal

    // Beep if silent mode not set,  if either we have a short on the main output, beeps is set by the op flags (error condition), or if the alarm input
    // from the booster is active whilst booster track power is on

    AWD = !NVPtr->userflags.silent && ((main_retry_delay > 0) || op_flags.beeping  || (!ALARM && op_flags.op_pwr_m ));

#elif CANCSB
    // AWD drive signal

    // Beep if silent mode not set,  if either we have a short on the main output, beeps is set by the op flags (error condition), or if the alarm input
    // from the booster is active whilst booster track power is on

    AWD = !NVPtr->userflags.silent && ((main_retry_delay > 0) || op_flags.beeping  || (!ALARM && op_flags.op_pwr_m ));    

#elif CANCMDB
    // AWD drive signal

    // Beep if silent mode not set,  if either we have a short on the main output, beeps is set by the op flags (error condition), or if the alarm input
    // from the booster is active whilst booster track power is on

    AWD = !NVPtr->userflags.silent && ((main_retry_delay > 0) || op_flags.beeping  || (!ALARM && op_flags.op_pwr_m ));    
    
#endif
}



// Set flag for whether on board output is the main or programming track (and the booster output the opposite)
// Returns true if the swap input, or swap flag depending on the hardware, is set.


#pragma tmpdata ISRHTMP  // Code from here on only called by ISR, use named tmpdata section to remove need to save tmpdata on context switch

BOOL set_output(ModNVPtr NVPtr)

{
#if BC1a
    return(SWAP_OP);   // fixed on BC1a
#elif CANCMD
    return( (NVPtr->opflags.j7ctrl && SWAP_OP) || !(NVPtr->opflags.j7ctrl || NVPtr->opflags.mainonboard) );
#elif CANCSB
    return(SWAP_OP);   // fixed on CANCSB
#elif CANCMDB
    return(SWAP_OP);   // same as CANCSB	
#endif
}



//
// Interrupt Service Routine
// TMR0 generates a heartbeat every 58uS to generate timing for the DCC bit
// stream. If no DCC packet to be transmitted then preamble is generated.
//
// One bit half cycle is 58usec. Zero bit half cycle is 116usec.
//
// A/D readings are stored in RAM.
//
#pragma interrupt isr_high nosave=section( ".tmpdata" ),section("MATH_DATA")
void isr_high(void) 
{
    // 29 clocks to get here after interrupt

    INTCONbits.T0IF = 0;
    TMR0L = tmr0_reload;
   
    ISR_PIN = 1;		// flag start of ISR - enable for debugging with logic analyser   
    
    
    if (tmr0_reload == TMR0_RAILCOM)  // Check if short "half normal interrupt period" timer reload has been set
    {
        if (railcomFlags.waitHalfPeriod)
            railcomFlags.waitHalfPeriod = FALSE;
        else // already waited for half period, so we are now at half period, 29uS after bit start
        {    
            dcc_flags.railcom_cutout_active = TRUE;  // Start cutout now as we are at half period time, next interrupt will be after another half period (ie: end of full period) but then reload normal 
            railcomFlags.atHalfPeriod = TRUE;           
            tmr0_reload = TMR0_NORMAL;
        }             
    }    
    else
        railcomFlags.atHalfPeriod = FALSE;
 
    // The following is for all except CANCSB and CANCMDB (replaces a #ifndef CANCSB)
#if BC1a
    if (dcc_flags.dcc_test_outputs)
        ISR_PIN = 1;		// flag start of ISR
    
#elif CANCMD
    if (dcc_flags.dcc_test_outputs)
        ISR_PIN = 1;		// flag start of ISR

#endif
    
    //
    // TMR0 is enabled all the time and we send a continuous preamble
    // as required for booster operation. The power may, however, be off
    // resulting in no actual DCC output
    //
    // Outputs are toggled here to set the output to the state
    // determined during the previous interrupt. This ensures the output
    // always toggles at the same time relative to the interrupt, regardless
    // of the route taken through the switch() statement for bit generation.
    //
    // The cancmd hardware does not use MOSFETs and does not need shoot-through
    // protection delay, but the BC1a does.
    //
    // When railcom is enabled, the interrupt period will be halved when the next bit is the first preamble, 
    // to allow the cutout to start half a bit time (29uS) after the last stop bit.  The setting
    // the outputs off to start the cutout is the only processing done at half normal interrupt period times.
    //


#if BC1a

	// BC1a FETs are always main track output
	// The original BC1a code runs at 16MHz, 4 cycles (250nS) per instruction
	// and has 16 instructions between changing over the DCC outputs (for the slower FETs)
	// This is a shoot-through delay of 4uS.

	// This firmware runs at 32MHz, so each instruction is 125nS.

    // Each time round the while loop is 4 instruction cycles, (3 on the last time round)
    // so the delay is 500ns x shoot-through NV value 
	// default is therefore a value of 8, but will be tuneable via FLiM
    

    sthru_count = sthru_delay;

    if (op_flags.op_pwr_m && !(dcc_flags.railcom_cutout_active)) 
    {
    	if (op_flags.op_bit_m) {
            DCC_NEG = 0;
            while (--sthru_count != 0) {}
            DCC_POS = 1;
    	} else {
            DCC_POS = 0;
            while (--sthru_count != 0) {}
            DCC_NEG = 1;
    	}
	} else {
            DCC_POS = 0;
            DCC_NEG = 0;
	}

    
    // 6202 on daughter board is service mode output

    if (!half_period)
    {    
        if (op_flags.op_pwr_s) {
            DCC_SVC_EN = 1;
            if (op_flags.op_bit_s) {
                DCC_SVC_NEG = 0;
                DCC_SVC_POS = 1;
            } else {
                DCC_SVC_POS = 0;
                DCC_SVC_NEG = 1;
            }
        } else {
            DCC_SVC_EN = 0;
            DCC_SVC_POS=0;
            DCC_SVC_NEG=0;
        }
    

    // Immediate check for digital overload detection
    
//    if (OLOAD_DETECTN == 0) 
//    {
//        // Dead short - shutdown immediately
//        DCC_POS = 0;
//        DCC_NEG = 0;
//        OVERLOAD_PIN = 1;
//        // start retry delay
//        op_flags.op_pwr_m = 0;
//        main_retry_delay = RETRY_DELAY;      // About half a second
//    }

    
        // Diagnostic outputs 

        // LED1Y is yellow LED flashes when programming track active
        // LED2G is green LED (blinks when processing commands)

        if (op_flags.op_pwr_s) {
            LED1Y = 1;
        } else {
            LED1Y = 0;
        }
    } // if not a half period


#elif CANCMD
    

        
    if (!swap_op)
    {
        // J7 is in
        // On board output is main track output        
  
        if (dcc_flags.railcom_cutout_active)
        {
           DCC_EN = 1;
           DCC_POS = 0;
           DCC_NEG = 0; 
        }        
        else if (!railcomFlags.atHalfPeriod) // not at half period - normal processing
        {    
            if (op_flags.op_pwr_m ) 
            {
                DCC_EN = 1;

                if (op_flags.op_bit_m) {
                    DCC_NEG = 0;
                    DCC_POS = 1;
                } else {
                    DCC_POS = 0;
                    DCC_NEG = 1;
                }
            } else {
                DCC_EN = 0;
                DCC_POS = 0;
                DCC_NEG = 0;
            }
        }    
         
        if (!railcomFlags.atHalfPeriod)
        {    
            // Booster output is service mode packets
            if (op_flags.op_pwr_s) {
                SHUTDOWN = 1;
                DCC_OUT = op_flags.op_bit_s;
            }
            else
                SHUTDOWN = 0;
            
            // J7 is in
            // On board booster is main track output
            // Yellow LED normally on, off during overload
            if (op_flags.op_pwr_m) {
                LED1Y = 1;
            } else {
                LED1Y = 0;
            }
            
            // In case mode is changed on the fly, we don't check for ack
            // when on board booster is main track output
  
            dcc_flags.dcc_check_ack = 0;
        } // not half period
    } // J7 in
    else    
    {
        // J7 is out
        // On-board output is service mode output
        
        if (!railcomFlags.atHalfPeriod)
        {    
            if (op_flags.op_pwr_s) {
            DCC_EN = 1;
                if (op_flags.op_bit_s) {
                    DCC_NEG = 0;
                    DCC_POS = 1;
                } else {
                    DCC_POS = 0;
                    DCC_NEG = 1;
                }
            } else
            {
                DCC_EN = 0;
                DCC_POS = 0;
                DCC_NEG = 0;
            }
            // Booster output is main track output
            if (op_flags.op_pwr_m)
            {
                SHUTDOWN = 1;
                DCC_OUT = op_flags.op_bit_m;
            }
            else
                SHUTDOWN = 0;
            
            // Power LEDs - J7 is out
            // On-board booster is service mode output
            // LED1 flashes on yellow whilst track is powered during programming
            if (op_flags.op_pwr_s) {
                LED1Y = 1;
            } else {
                LED1Y = 0;
            }
        } // not at half period
    } // else J7 out 
    
#elif CANCSB
    
	// L6203 is always main track output    
    
    if (railcomFlags.atHalfPeriod)
    {    
        if (dcc_flags.railcom_cutout_active)
        {
           DCC_EN = 1;
           DCC_POS = 0;
           DCC_NEG = 0; 
        }        
    }    
    else  // not at half period
    {    
        if (op_flags.op_pwr_m) 
        {
            SHUTDOWN = 1;                   // Enable booster output
            DCC_OUT = op_flags.op_bit_m;    // Booster output state

            if (dcc_flags.railcom_cutout_active)
            {
               DCC_EN = 1;
               DCC_POS = 0;
               DCC_NEG = 0; 
            }
            else // cutout not active
            {    
                DCC_EN = 1;                     // Enable main output
                if (op_flags.op_bit_m) {
                    DCC_NEG = 0;                // Main track output state
                    DCC_POS = 1;                // Main track output state
                } else {
                    DCC_POS = 0;                // Main track output state
                    DCC_NEG = 1;                // Main track output state
                }
            }
        } else // main track power off
        {
            DCC_EN = 0;
            DCC_POS = 0;
            DCC_NEG = 0;
            SHUTDOWN = 0;
        }    
  

        // L6202 is always service track output
    
        if (op_flags.op_pwr_s) {
            DCC_SVC_EN = 1;
            if (op_flags.op_bit_s) {
                DCC_SVC_NEG = 0;
                DCC_SVC_POS = 1;
            } else {
                DCC_SVC_POS = 0;
                DCC_SVC_NEG = 1;
            }
        } else {
            DCC_SVC_EN = 0;
            DCC_SVC_POS=0;
            DCC_SVC_NEG=0;
        }

        // LED1Y is yellow LED flashes when programming track active
        // LED2G is green LED (blinks when processing commands)

        if (op_flags.op_pwr_s) {
            LED1Y = 1;
        } else {
            LED1Y = 0;
        }
    }
    
#elif CANCMDB
    
	// L298 is dual bridge so there is always a main track output    
    sthru_count = sthru_delay;
    
    if (railcomFlags.atHalfPeriod)
    {    
        if (dcc_flags.railcom_cutout_active)
        {
           DCC_EN = 1;
           DCC_POS = 0;
           DCC_NEG = 0;
           RCUT = 1;           //turn on railcom cutout FETs
        //   toggle_dcc_m();   //invert bits so output will be opposite on exit from railcom cutout
        }        
    }    
    else  // not at half period
    {    //SW added shoothru delay capability
        if (op_flags.op_pwr_m) 
        {
            SHUTDOWN = 1;                   // Enable booster output
            DCC_OUT = op_flags.op_bit_m;    // Booster output state

            if (dcc_flags.railcom_cutout_active)
            {
               DCC_EN = 1;
               DCC_POS = 0;
               DCC_NEG = 0;
               RCUT = 1;           //turn on railcom cutout FETs 
            }
            else // cutout not active
            {    
                RCUT = 0;          //turn off railcom cutout FETs
                DCC_EN = 1;
                if (op_flags.op_bit_m) {
                DCC_NEG = 0;
                while (--sthru_count != 0) {}
                DCC_POS = 1;
                } else {
                  DCC_POS = 0;
                  while (--sthru_count != 0) {}
                  DCC_NEG = 1;
                }
            }         
        } else // main track power off
        {
            RCUT = 0;           //ensure railcom cutout FETs are off
            DCC_EN = 0;
            DCC_POS = 0;
            DCC_NEG = 0;
            SHUTDOWN = 0;
        }    
  

        // L298 second bridge is always service track output
    
        if (op_flags.op_pwr_s) {
            DCC_SVC_EN = 1;
            if (op_flags.op_bit_s) {
                DCC_SVC_NEG = 0;
                DCC_SVC_POS = 1;
            } else {
                DCC_SVC_POS = 0;
                DCC_SVC_NEG = 1;
            }
        } else {
            DCC_SVC_EN = 0;
            DCC_SVC_POS=0;
            DCC_SVC_NEG=0;
        }

        // LED1Y is yellow LED flashes when programming track active
        // LED2G is green LED (blinks when processing commands)

        if (op_flags.op_pwr_s) {
            LED1Y = 1;
        } else {
            LED1Y = 0;
        }
    }
    
#endif // CANCMDB

    if (!railcomFlags.atHalfPeriod)
    {
        setbeep((ModNVPtr) NVPtr);

        // Determine next service mode track DCC output bit
        switch(bit_flag_s) {
            case FIRST_EDGE:
                toggle_dcc_s();
                // test for a 1 or 0 to transmit
                bit_flag_s = ((next_byte_s & 0x80) == 0x80) ? SECOND_EDGE_BIT1 : FIRST_MIDPOINT_BIT0;
                break;

            case SECOND_EDGE_BIT1:
                toggle_dcc_s();
                // fall through to next_bit

            case SECOND_MIDPOINT_BIT0:
                bit_flag_s = FIRST_EDGE;			// likely to be first edge next
                next_byte_s = next_byte_s << 1;		// Rotate DCC data byte for next bit
                dcc_bits_s--;
                if (dcc_bits_s != 0) {
                    break;
                }
                dcc_idx_s++;		                    // all bits done so point to next byte
                next_byte_s = dcc_buff_s[dcc_idx_s];
                dcc_bits_s = 8;		                // reload bit counter
                byte_cnt_s--;
                if (byte_cnt_s == 0) {
                    dcc_buff_s[6]--;                  // no more bytes, more packets?
                    if ((dcc_buff_s[6] == 0)
                        || ((dcc_flags.dcc_rec_time == 0) && (dcc_flags.dcc_ack == 1))) {
                        bit_flag_s = FIRST_IDLE;		// no more packets or ack received
                                                        // and not recovery time
                        dcc_flags.dcc_rdy_s = 1;		// buffer free
                    } else {
                        bit_flag_s = FIRST_PRE;       // do another packet
                        dcc_idx_s = 0;			    // reset pointer
                        next_byte_s = dcc_buff_s[dcc_idx_s];
                        pre_cnt_s = dcc_pre_s;
                        byte_cnt_s = dcc_bytes_s;
                        dcc_bits_s = 8;               // and bit count
                    }
                } else {
                    bit_flag_s = FIRST_START;		    // start another byte
                }
                break;

            case SECOND_EDGE_BIT0:
                toggle_dcc_s();
                bit_flag_s = SECOND_MIDPOINT_BIT0;
                break;

            case FIRST_MIDPOINT_BIT0:
                bit_flag_s = SECOND_EDGE_BIT0;
                break;

            case SECOND_IDLE:
                toggle_dcc_s();
                bit_flag_s = FIRST_IDLE;
                // test for new packet
                if (dcc_flags.dcc_rdy_s == 0) {
                    bit_flag_s = FIRST_PRE;
                    dcc_idx_s = 0;			    // set up pointer
                    next_byte_s = dcc_buff_s[dcc_idx_s];
                    pre_cnt_s = dcc_pre_s;
                    byte_cnt_s = dcc_bytes_s;
                    dcc_bits_s = 8;
                }
                break;

            case FIRST_IDLE:
                toggle_dcc_s();
                bit_flag_s = SECOND_IDLE;
                break;

            case SECOND_PRE:
                toggle_dcc_s();
                // check if ready for start bit
                pre_cnt_s--;
                if (pre_cnt_s == 0) {
                    bit_flag_s = FIRST_START;
                } else {
                    bit_flag_s = FIRST_PRE;
                }
                break;

            case FIRST_PRE:
                // Could keep track of how many preamble bits have been sent with power on
                // but for now we assume we are starting from scratch
                toggle_dcc_s();
                bit_flag_s = SECOND_PRE;
                break;

            case SECOND_MID_START:
                bit_flag_s = FIRST_EDGE;
                break;

            case SECOND_START:
                toggle_dcc_s();
                bit_flag_s = SECOND_MID_START;
                break;

            case FIRST_MID_START:
                bit_flag_s = SECOND_START;
                break;

            case FIRST_START:
            default:
                toggle_dcc_s();
                bit_flag_s = FIRST_MID_START;
                break;
        } // switch(bit_flag_s)

        //
        // Determine next main track DCC output bit
        //
        switch(bit_flag_m) {
            case FIRST_EDGE:
                toggle_dcc_m();
                // test for a 1 or 0 to transmit
                bit_flag_m = ((next_byte_m & 0x80) == 0x80) ? SECOND_EDGE_BIT1 : FIRST_MIDPOINT_BIT0;
                break;

            case SECOND_EDGE_BIT1:
                toggle_dcc_m();
                // fall through to next_bit

            case SECOND_MIDPOINT_BIT0:
                bit_flag_m = FIRST_EDGE;			// likely to be first edge next
                next_byte_m = next_byte_m << 1;		// Rotate DCC data byte for next bit
                dcc_bits_m--;
                if (dcc_bits_m != 0) {
                    break;
                }

                if ((dcc_idx_m == 0) && dcc_flags.dcc_test_outputs)
                {
                  DCC_PKT_PIN = 1;					// Spare output pin for sync scope on start of packet
                }

                dcc_idx_m++;		                    // all bits done so point to next byte
                next_byte_m = dcc_buff_m[dcc_idx_m];
                dcc_bits_m = 8;		                // reload bit counter
                byte_cnt_m--;
                if (byte_cnt_m == 0)
                {
                   if (dcc_flags.dcc_test_outputs)
                       DCC_PKT_PIN = 0;
                    dcc_buff_m[6]--;                  // no more bytes, more packets?
                    if ((dcc_buff_m[6] == 0)) {
                        bit_flag_m = FIRST_IDLE;		// no more packets
                        dcc_flags.dcc_rdy_m = 1;		// buffer free
                    } else {
                        bit_flag_m = FIRST_PRE;       // do another packet
                        dcc_idx_m = 0;			    // reset pointer
                        next_byte_m = dcc_buff_m[dcc_idx_m];
                        pre_cnt_m = dcc_pre_m;
                        byte_cnt_m = dcc_bytes_m;
                        dcc_bits_m = 8;               // and bit count 
                    }
                } else {
                    bit_flag_m = FIRST_START;		    // start another byte
                }
                break;

            case SECOND_EDGE_BIT0:
                toggle_dcc_m();
                bit_flag_m = SECOND_MIDPOINT_BIT0;
                break;

            case FIRST_MIDPOINT_BIT0:
                bit_flag_m = SECOND_EDGE_BIT0;
                break;

            case SECOND_IDLE:
                toggle_dcc_m();
                bit_flag_m = FIRST_IDLE;
                // test for new packet
                if (dcc_flags.dcc_rdy_m == 0) {
                    bit_flag_m = FIRST_PRE;
                    dcc_idx_m = 0;			    // set up pointer
                    next_byte_m = dcc_buff_m[dcc_idx_m];
                    pre_cnt_m = dcc_pre_m;
                    byte_cnt_m = dcc_bytes_m;
                    dcc_bits_m = 8;
                }
                break;

            case FIRST_IDLE:
                toggle_dcc_m();
                bit_flag_m = SECOND_IDLE;
                break;

            case SECOND_PRE:
                toggle_dcc_m();
                // check if ready for start bit
                pre_cnt_m--;
                if (pre_cnt_m == 0) {
                    bit_flag_m = FIRST_START;
                } else {
                    bit_flag_m = FIRST_PRE;
                }
                break;

            case FIRST_PRE:
                // Could keep track of how many preamble bits have been sent with power on
                // but for now we assume we are starting from scratch
                toggle_dcc_m();
                if (railcomFlags.railcomEnabled)
                {   
                    if (pre_cnt_m == dcc_pre_m)  // For first preamble, initiate wait for half period (29uS) before starting Railcom cutout)
                    {
                        tmr0_reload = TMR0_RAILCOM;  // Next time reload timer with half bit time
                        railcomFlags.waitHalfPeriod = TRUE;
                    }    
                    if (pre_cnt_m == DCC_CUTOUT_END)  // End Railcom cutout by checking number of preambles left.
                    {    
                        dcc_flags.railcom_cutout_active = FALSE;
                    }    
                }
                bit_flag_m = SECOND_PRE;
                break;

            case SECOND_MID_START:
                bit_flag_m = FIRST_EDGE;
                break;

            case SECOND_START:
                toggle_dcc_m();
                bit_flag_m = SECOND_MID_START;
                break;

            case FIRST_MID_START:
                bit_flag_m = SECOND_START;
                break;

            case FIRST_START:
            default:
                toggle_dcc_m();
                bit_flag_m = FIRST_MID_START;
                break;
        } // switch(bit_flag_m)

        //
        // Current sensing
        //
        // Acquisition time is within one DCC half bit period (58us).
        //
        // Conversion time is 11 * Tad. Tad is Tosc*64 or 2.66us at 24MHz. So,
        // conversion time is also within one DCC half bit period.
        //
        // one state covers aquisition time, a second state the conversion time.
        // Code cycles through two channels even though some hardware only uses one
        // from the current shunt) so
        // the sample period for each channel is 58us *2 *2 = 232us.
        //
        // Current sense samples are averaged over four sample
        // periods.
        //
        // If a potential overload condition is sensed, a timer is started to
        // check gain after 130 cycles or approx 30ms. If the overload is still present
        // in programming mode then DCC_FLAGS, DCC_OVERLOAD is set.
        //
        switch(ad_state) {
            case AD_RESET:		// 0
                ad_state = ACQ_MAIN;	// ADCON0 already intialised to main mode during intialisation
                break;

            case ACQ_MAIN:
                ad_state = CONV_MAIN;
                if ((analogue_main) && (!swap_op)) {
                  ADCON0bits.GO = 1;
                }
                break;

            case CONV_MAIN:
                if (!swap_op) {    // on board booster mode
                    if (analogue_main) {
                        if (ADCON0bits.GO == 0) {
                            anmain = ((unsigned int)ADRESH<<8) | ADRESL;
                            // Averaging function used for current sense is
                            // S(t) = S(t-1) - [S(t-1)/N] + VIN
                            // A(t) = S(t)/N
                            //
                            // where S is an accumulator, N is the length of the filter (i.e.,
                            // the number of samples included in the rolling average), and A is
                            // the result of the averaging function.
                            // The time constant of the averaging equation is given by t = N/FS,
                            // where N is again the length of the filter and FS is the sampling
                            // frequency.
                            //
                            // Re-arranged for N=4:
                            // S(t) = S(t-1) - A(t-1) + Vin
                            // A(t) = S(t)/4
                            summain = summain - avemain;            // S(t) = S(t-1) - A(t-1)
                            summain = summain + anmain;            // S(t) = S(t-1) - A(t-1) + Vin
                            avemain = summain>>2;               // A(t) = S(t)/4
                            ADCON0 = ADCON0_SVC;				// Switch to service mode channel
                             ad_state = ACQ_SVC;
                        } // if A->D conversion complete
                    } // if analogue main track overload detection
                    else { // digital main track overload detection
                        ADCON0 = ADCON0_SVC;				// Switch to service mode channel
                    ad_state = ACQ_SVC;
                    }

                     // Are we waiting for retry delay?
                    if (main_retry_delay == 0)
                    {
                        // No, have we had a potential overload?
                        if (main_ovld_delay == 0)
                        {
                            // No, do normal checks
                            // check for dead short immediately
                            // current is limited by LM317
                            if (((short)ilimitmain < anmain) || (OLOAD_DETECTN == 0) ) 
                            {
                                // Dead short - shutdown immediately
                                DCC_POS = 0;
                                DCC_NEG = 0;
                                if (dcc_flags.dcc_test_outputs)
                                    OVERLOAD_PIN = 1;
                            // start retry delay
                            op_flags.op_pwr_m = 0;
                            main_retry_delay = RETRY_DELAY;      // About half a second
                            }
                else
                            { // not dead short, look for overload
                                if ((avemain >= (short)imaxmain) || (OLOAD_DETECTN == 0))
                                {
                                    main_ovld_delay = OVLD_DELAY;         // 30ms delay
                                }
                            }
                        }
                        else
                        {   // main track overload detection delay in progress
                            // countdown overload delay
                            main_ovld_delay--;
                            if (main_ovld_delay == 0) {
                                // Check again
                                if ((avemain >= (short)imaxmain) || (OLOAD_DETECTN == 0)) {
                                    // still overload so power off
                                    if (dcc_flags.dcc_test_outputs)
                                        OVERLOAD_PIN = 1;				// Set spare output pin for scope monitoring
                                    // low power booster mode restart retry delay
                                    op_flags.op_pwr_m = 0;
                                    main_retry_delay = RETRY_DELAY;          // 9 * 256 * 232us ~ 535ms
                                }
                            }
                        }
                    } else 
                    {   // main track retry delay in progress
                        // countdown retry delay
                        main_retry_delay--;
                        if (main_retry_delay == 0) {
                           // Request power on again
                            dcc_flags.dcc_retry = 1;
                            if (dcc_flags.dcc_test_outputs)
                                OVERLOAD_PIN = 0;
                        }
                    }
                } // if on board booster
                else {
                    ADCON0 = ADCON0_SVC;				// Switch to service mode channel
                    ad_state = ACQ_SVC;
                }
                break;

            case ACQ_SVC:
                ad_state = CONV_SVC;
                ADCON0bits.GO = 1;
                break;


            case CONV_SVC:
                if (ADCON0bits.GO == 0) {
                    ansvc = ((unsigned int)ADRESH<<8) | ADRESL;
                    // The averaging code is duplicated rather than extracted as a function with parameters to
                    // avoid the C calling overhead of a function in the high priority ISR

                    sumsvc = sumsvc - avesvc;            // S(t) = S(t-1) - A(t-1)
                    sumsvc = sumsvc + ansvc;            // S(t) = S(t-1) - A(t-1) + Vin
                    avesvc = sumsvc>>2;               // A(t) = S(t)/4
                    ADCON0 = ADCON0_MAIN;				// Switch back to main track channel
                    ad_state = ACQ_MAIN;

    #ifdef CANCMD
                    if (swap_op) 	// Service mode output active
    #endif
                    {
                        // Have we had a potential overload?
                        if (svc_ovld_delay == 0) {
                            // No, do normal checks
                            if (dcc_flags.dcc_check_ack == 1) {
                                // has current increased, denoting ACK?
                                if (avesvc >= (short)(iccq + iackdiff)) {
                                    dcc_flags.dcc_ack = 1;
                                    //	ACK_PIN = 1;
                                }
                            } else {
                                // check for dead short immediately
                                // current is limited by LM317
                                if ((short)ilimitsvc > ansvc) {
                                    // not dead short, look for overload
                                    if ((dcc_flags.dcc_check_ovld == 1) || (!swap_op)) {         // low power booster mode
                                        if (avesvc >= (short)imaxsvc) {
                                            svc_ovld_delay = OVLD_DELAY;         // 130*232us = 30160us delay
                                        }
                                    }
                                } else {
                                    // Dead short - shutdown immediately
                                    if (dcc_flags.dcc_test_outputs)
                                        OVERLOAD_PIN = 1;
                                    // programming mode so react immediately
                                    op_flags.op_pwr_s = 0;
                                    dcc_flags.dcc_overload = 1;
                                    dcc_flags.dcc_check_ovld = 0;
                                }
                            }
                        } else { // overload detection delay in progress
                            // countdown overload delay
                            svc_ovld_delay--;
                            if (svc_ovld_delay == 0) {
                                // Check again
                                if (avesvc >= (short)imaxsvc) {
                                    // still overload so power off
                                    if (dcc_flags.dcc_test_outputs)
                                        OVERLOAD_PIN = 1;				// Set spare output pin for scope monitoring
                                    // programming mode so react immediately
                                    op_flags.op_pwr_s = 0;
                                    dcc_flags.dcc_overload = 1;
                                    dcc_flags.dcc_check_ovld = 0;
                                }
                            }
                        } // else overload delay
                    } // if service output active
                } // if A->D conversion complete
                break;

            default:
                break;
        } // switch(ad_state)

        //
        // Slot timeout and other timers - every half second
        //
        if (--slot_timer == 0) {
            slot_timer = ((short long)HEARTBEAT)/DCC_BASE_PERIOD;    
            op_flags.slot_timer = 1;

            if (canTransmitTimeout != 0) {
                    --canTransmitTimeout;
            }

            swap_op = set_output( (ModNVPtr) NVPtr );  // Check if jumper has been changed
        }   

    } // if !half period
    
    ISR_PIN = 0;		// flag end of ISR
    
// for all except CANCSB and CANCMDB for debugging (was a #ifndef CANCSB)
#if BC1a
    if (dcc_flags.dcc_test_outputs)
        ISR_PIN = 1;		// flag start of ISR
    
#elif CANCMD
    if (dcc_flags.dcc_test_outputs)
        ISR_PIN = 0;		// end of ISR

#endif
    
} // isr_high

#pragma tmpdata // revert to the default .tmpdata section


