/*
 
   Copyright (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 This code is for a CBUS DCC Command Station
 
 FLiM related routines specific to the command station

These routines implement the behaviour in response to events
The node variable and event table storage is declared here
  
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
	30/5/11	Pete Brownlow	- Initial outline	
 
 For version number and full revision history see CANCMD.c
 
*/ 

#include "project.h"


#pragma romdata PARAMETERS

#define PRM_CKSUM MANU_ID+MINOR_VER+MODULE_ID+MAX_EVT+EVperEVT+NV_NUM+MAJOR_VER+MODULE_FLAGS+CPUID+PB_CAN+(LOAD_ADDRESS>>8)+(LOAD_ADDRESS&0xFF)+CPUM_MICROCHIP+BETA+sizeof(ParamVals)+(MNAME_ADDRESS>>8)+(MNAME_ADDRESS&0xFF)


const rom ParamVals     FLiMparams = { MANU_ID, MINOR_VER, MODULE_ID, MAX_EVT, EVperEVT , NV_NUM, MAJOR_VER, MODULE_FLAGS, CPUID,PB_CAN,LOAD_ADDRESS,0,CPUM_MICROCHIP,BETA };
const rom SpareParams    spareparams = {0,0,0,0};
const rom FCUParams     FCUparams   = { sizeof(ParamVals),(DWORD)&module_type,(WORD)PRM_CKSUM };
const rom char          module_type[] = MODULE_TYPE;


// Node and event variables at a fixed place in ROM, starting on a segment boundary
// so they can be written to as required 

#pragma romdata	FLIMDATA	// Node and event variables

// const rom EventTableEntry eventTable[MAX_EVT];  // Events will be supported in version 5
const rom NodevarTable	nodevartable = { DEFAULT_CS_NUM, DEFAULT_USER_FLAGS, DEFAULT_OP_FLAGS, DEBUG_FLAGS, DEFAULT_WALK_TIMEOUT, MAIN_CURRENT_LIMIT ,
                                         SVC_CURRENT_LIMIT, MAIN_CURRENT_MULT, SVC_ACK_DIFF, SHOOTTHRU_DELAY,0,0,AMPERAGE_REPORTING,DEFAULT_SOD_DELAY,DEFAULT_HONK_INTERVAL,DEFAULT_MAX_SPEED,


// Test values for debugging shuttles
#if DEBUG_SHUTTLES
                                       68,0,10,0x85};
#elif FFQ_SHUTTLES
                                       3,200,32,0x85,83,197,40,0x85,7,192,28,0x85,136,192,50,0x85};
#else
                                        };
#endif


const rom BYTE topofFLiM = 99;

#pragma romdata


// Static RAM variables 

#pragma udata SHUTTLES

ActiveShuttleEntry	activeShuttleTable[ MAX_SHUTTLES ];

#pragma udata DELEVT

DelayListEntry          delayedEvents[ MAX_DELAYED_EVENTS ];

#pragma udata MAIN_VARS

ModNVPtr cmdNVptr;                      // Pointer to node variables structure
ModEVPtr cmdEVptr;                      // Pointer to event structure
BOOL	NV_changed;
BOOL	FLiMPressed;			// Flag for FLiM button pressed
BYTE	FLiMHoldCount;			// Flim hold timer
BOOL	FLiMFlash;			// LED is flashing
BOOL	FlashStatus;			// Control flash on/off of LED during FLiM setup
BYTE    AmperageCountdown;              // Countdown to next amperage message
StatFlags CSStatus;                 // Status flags

#pragma udata 

//#pragma romdata EEPROM
// rom BYTE eedata_vals[8] = {DEFAULT_NN & 0x00FF, DEFAULT_NN << 8, 0, 0, 0, 0, 0, 0};
//#pragma romdata



#pragma code APP


// cmdFLimInit called during intialisation 

void	cmdFlimInit(void)

{
	BYTE        i;
	
  	FLiMinit();
 
        // Initialise node variable and events table pointers

	NVPtr = &(nodevartable.nodevars[0]);         // Node Variables table
    cmdNVptr = (ModNVPtr) NVPtr;
//	EVTPtr = eventTable;                         // Events will be supported in version 5
//  cmdEVptr = (ModEVPtr) EVTPtr;
	NV_changed = FALSE;
	FLiMPressed = FALSE;
	FLiMFlash = FALSE;
    FlashStatus = FALSE;

    AmperageCountdown = cmdNVptr->sendcurrentinterval << 1; // Message interval in half second heartbeats

    initShuttles(cmdNVptr);
    
} // cmdFlimInit

void sendStartOfDay( ModNVPtr cmdNVPtr )
{
    if (cmdNVPtr->userflags.startofday)
        sendCbusEvent( 1, TRUE );    // Default for start of day is long event 1. Version 5 will support teaching events including this one.
}

// Check for FLiM button pressed and control entry into FLiM setup mode

void FLiMSWCheck( void )

{

    if (FLiMFlash)
      FlashStatus = !FlashStatus;

    if (!SW)    // FLiM button pressed
    {
        switch (flimState)
        {
            case fsSLiM:
                        FLiMHoldCount = 0;      // start counting how long button is held down for
                        flimState = fsPressed;
                        break;

            case fsPressed:
                        if (++FLiMHoldCount >= FLIM_HOLD_COUNT)
                        {
                            FLiMFlash = TRUE;
                            flimState = fsFlashing;
                        }
                        break;

            case fsFLiM:
            case fsFLiMLearn:
                        FLiMHoldCount = 0;      // start counting how long button is held down for
                        flimState = fsPressedFLiM;
                        break;

            case fsPressedFLiM:
                        FLiMHoldCount++;        // What happens will depend on how long when released
                        break;

            case fsFLiMSetup:
                        flimState = fsPressedSetup;
                        break;
            default:
                        break;
        }  // switch
    }
    else // button not pressed
    {
        switch (flimState)
        {
            case fsSLiM:
            case fsFLiM:
                        break;              // Most comon case, nOthing to do - put at top for faster execution (wot - don't you trust the compiler? !!)

            case fsPressed:
                        flimState = fsSLiM; // Button not pressed long enough                        {
                        break;

            case fsFlashing:
                        FLiMsetup();  // Switch pressed for long enough so enter FLiM setup mode
                        flimState = fsFLiMSetup;
                        break;

            case fsSetupDone:
                        FLiMFlash = FALSE;
                        flimState = fsFLiM;
                        break;

            case fsPressedFLiM:
                        if (FLiMHoldCount > 0)
                        {
                            if (FLiMHoldCount > FLIM_HOLD_COUNT)
                                SLiMrevert();   // long press in FLiM reverts to SLiM
                            else
                            {
                                FLiMFlash = TRUE;
                                FLiMsetup();  // Short press in FLiM re-enters setup mode
                            }
                        }
                        break;

            case fsPressedSetup:                // Half second press in setup ends setup
                        FLiMEndSetup();
                        FLiMFlash = FALSE;
                        break;

            default:
                        break;
        }
    }
   
} // FLiMSWCheck



// Blink green LED on to show busy doing commands, but overidden by flash if in FLiM setup mode

BYTE BlinkLED( BOOL blinkstatus )


{
	BYTE LEDstatus;

	if (FLiMFlash)
	{
		LEDstatus = FlashStatus;
		// FlashStatus = !FlashStatus;
	}
	else
		LEDstatus = blinkstatus;

	return( LEDstatus ? 1 : 0 );
	
} // BlinkLED


// Check if we need to send a CBUS message for amperage

void amperage_messages(void)
{
    WORD_VAL  amperage;
    BYTE    multiplier;
    BYTE    AtoDvalue;


    if ( AmperageCountdown != 0 )     // 0 means feature disabled
    {
        if (--AmperageCountdown == 0)
        {
            AmperageCountdown = cmdNVptr->sendcurrentinterval; // Message interval in half second heartbeats

            multiplier = cmdNVptr->currentmult;
            AtoDvalue = (avemain & 0x00FF);
            amperage.Val = AtoDvalue * multiplier;
            
 
            // The above is an 8x8 multiply, but I could not persuade the compiler to give the 16 bit result, despite the
            // fact that it uses the PIC 8x8 multipy instruction.
            // (promote or typecast either or both operands to 16 bit, and it does a long and painful 16x16 multiply)
            // So this sneaky line of code grabs the msbyte of the multiply just done and puts it in the right place

            amperage.byte.HB = PRODH;

            Tx1[d0] = OPC_ACON2;
            Tx1[d3] = 0;
            Tx1[d4] = AMPERAGE_EVENT;
            Tx1[d5] = amperage.byte.HB;
            Tx1[d6] = amperage.byte.LB;
            sendCbusMsgNN(Node_id);
        }
    }
    else
        AmperageCountdown = cmdNVptr->sendcurrentinterval;  // Reload interval so we pick up any NV change immediately
}


// Send command station status message

void send_stat( void )

{
    CSStatus.serviceModeOn = op_flags.op_pwr_s;
    CSStatus.trackOn = op_flags.op_pwr_m;  

    Tx1[d0] = OPC_STAT;
    Tx1[d3] = cmdNVptr->csnum;
    Tx1[d4] = CSStatus.byte;  // ??? put in cs flags as per opcode
    Tx1[d5] = MAJOR_VER;
    Tx1[d6] = MINOR_VER;
    Tx1[d7] = BETA;
    sendCbusMsgNN( Node_id );
 
}

// Sets a particular loco into a shuttle table entry
// Could be as a result of release to shuttle from a cab
// or from an RFID sensor

void set_shuttle_loco( BYTE session, BYTE shuttle_id )

{
    BYTE prev_shuttle_session;
    BYTE defaultSpeed;
    BYTE countValue;

    if ((shuttle_id < MAX_SHUTTLES) && (q_queue[session].status.valid == 1))
    {

        // Release any other loco in this shuttle
        if (activeShuttleTable[ shuttle_id ].flags.valid)
        {
            prev_shuttle_session = activeShuttleTable[ shuttle_id ].session;
            if (q_queue[prev_shuttle_session].status.valid)
            {
                q_queue[prev_shuttle_session].status.shuttle = FALSE;
                if (q_queue[prev_shuttle_session].status.dispatched)
                {
                    // Set previous loco shuttle speed to zero and timeout count to 1 so will timeout and release shortly
                    q_queue[prev_shuttle_session].speed &= 0x80;
                    q_queue[prev_shuttle_session].timeout = 1;
                }
            }
        }
            
        // Set this loco into the shuttle
        activeShuttleTable[ shuttle_id ].session = session;
        activeShuttleTable[ shuttle_id ].loco_addr = q_queue[session].address;


        if (nodevartable.module_nodevars.shuttletable[shuttle_id].flags.valid)
            defaultSpeed = nodevartable.module_nodevars.shuttletable[shuttle_id].default_speed;
        else
            defaultSpeed = 10;
                    
        activeShuttleTable[ shuttle_id ].set_speed = (((q_queue[session].speed & 0x7F) == 0) ?
        defaultSpeed  : q_queue[session].speed );

        activeShuttleTable[ shuttle_id ].counter = nodevartable.module_nodevars.honkInterval;
        
        activeShuttleTable[ shuttle_id ].flags.valid = TRUE;
        activeShuttleTable[ shuttle_id ].flags.started = TRUE;
        activeShuttleTable[ shuttle_id ].flags.manual = FALSE;
        activeShuttleTable[ shuttle_id ].flags.directionSet = FALSE;
        q_queue[session].status.shuttle = TRUE;
        q_queue[session].status.share_count += 1;       // Throttle that is releasing into here will still have a session at this point, which will decrement when it releases
        sendShuttleStatus(SHUTTLE_EVENT_SET, shuttle_id);
    }
} // set shuttle loco


// Puts a session into the specified shuttle entry

BOOL populate_shuttle( BYTE session, BYTE shuttle_id, BOOL ifempty )

{
    BOOL    populate;

    populate = !(activeShuttleTable[shuttle_id].flags.valid && ifempty);

    if (populate)
        set_shuttle_loco(session, shuttle_id);
    
    return( populate );
}

// Find a session in the active shuttle table
// Returns index to shuttle table, or 0xFF if not found

BYTE find_shuttle_session( BYTE session )

{
    BYTE    shuttle_index;

    for ( shuttle_index = 0; (shuttle_index < MAX_SHUTTLES) && (activeShuttleTable[ shuttle_index ].session != session); shuttle_index++ );

    if ( shuttle_index == MAX_SHUTTLES )
        shuttle_index = 0xFF;

    return( shuttle_index );
}

// Clear loco from shuttle

void clear_shuttle_entry( BYTE session )

{
    BYTE    shuttle_index;

    shuttle_index = find_shuttle_session( session );

    if (shuttle_index != 0xFF)
        activeShuttleTable[ shuttle_index ].flags.valid = 0;
}


void processDelayedEvents( void )

{
    BYTE i;
    
    for (i=0; i < MAX_DELAYED_EVENTS; i++)
    {
        if (delayedEvents[i].delayedEventIndex != 0xFF)
        {
            if (--delayedEvents[i].delayCount == 0)
            {
               processDelayedEvent(delayedEvents[i], (ModNVPtr) cmdNVptr);
               delayedEvents[i].delayedEventIndex = 0xFF;
            }
        }
    }    
        
}

BOOL addDelayedEvent ( BYTE eventIndex, BYTE delayCount, enum eventActions Action, BYTE param )

{
    BYTE i;

    for (i=0; (i < MAX_DELAYED_EVENTS) && (delayedEvents[i].delayedEventIndex != 0xFF); i++ );

    if (i < MAX_DELAYED_EVENTS)
    {        
      delayedEvents[i].delayCount = delayCount;
      delayedEvents[i].delayedEventIndex = eventIndex;
      delayedEvents[i].action = Action;
      delayedEvents[i].param = param;
    }
      
    return (i < MAX_DELAYED_EVENTS);
}
    

