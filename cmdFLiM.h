#ifndef __cmdFLiM_H
#define __cmdFLiM_H

/*
    Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

	Node variable and Event variable definitions for CBUS command station

	This header file contains the FLiM related defintions that are specific to the
	CBUS command station. CBUS Command Station - project definitions 

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
 
 18/11/18   PNB Add NV bit to enable TOTI digital inputs 
 20/5/19    PNB Add NV flag bit to enable an input for railcom cutout to work with Mike's RCCUT
 
*/ 


#define NV_NUM		sizeof(ModuleNodeDefs)  // 16+(4*MAX_HANDLES)	// Number of node variables
#define MAX_EVT		255                     // Number of events
#define EVperEVT	sizeof(SHTEvent)        // Event variables per event

#define	FLIM_HOLD_COUNT	6	// Number of hearbeats (half sec) required for FLiM button to be pressed

#define DEFAULT_CAN_ID    0x72
#define DEFAULT_NN 	0xFFFE

#define MAX_DELAYED_EVENTS  32

#include "FLiM.h"

typedef	union
{
    struct
    {
        BOOL 	hwError:1;
        BOOL	trackError:1;
        BOOL	trackOn:1;
        BOOL    busOn:1;
        BOOL    EMStopAllActive:1;
        BOOL    resetDone:1;
        BOOL    serviceModeOn:1;
        BOOL    reserved:1;
    } ;
    BYTE	byte;
} StatFlags;


//***************************************************************************************
// Data structures for Node variables table including shuttles
// ****************************************************************************************

// Data structures for shuttle table stored in node variables

// Shuttle flags structure used in both NV in flash and active table in RAM.
// The dynamic flags valid and started only have meaning in the active table
 
typedef	union
{
    struct
    {
        BOOL 	valid:1;
        BOOL	started:1;
        BOOL	autostart:1;
        BOOL    manual:1;
        BOOL    directionSet:1;
        BOOL    fwdDirBit:1;
    } ;
    BYTE	byte;
} ShuttleFlags;

typedef	struct
{
    dcc_address     loco_addr;
    BYTE            default_speed;
    ShuttleFlags    flags;
} ShuttleEntry;


// Data structures for active shuttle table

typedef	struct
{
    dcc_address     loco_addr;
    BYTE            set_speed;
    ShuttleFlags    flags;
    BYTE            session;		// session id for active shuttle
    BYTE            counter;        // For sequencing - in POC used for honk/whistle delay
} ActiveShuttleEntry;

typedef struct
{
    BYTE activeDebounceCount;
    BYTE inactiveDebounceCount;
} TOTIEntry;


// Data structures for Node Variables


typedef	union
{
    struct
    {
        BOOL	silent:1;           // Silent mode (no beeps)
        BOOL    PermitSteal:1;      // Set to 1 to enable steal feature
        BOOL    PermitShare:1;      // Set to 1 to enable share feature
        BOOL    PermitEventReset:1; // Set true to allow a taught event to reset the CS (was Speed must be zero to reverse direction, but this is better implemented in the cab)
        BOOL	mapdccacc:1;        // Set to map C bus event number to DCC accessory no.
        BOOL	stopontimeout:1;    // Bring loco to a halt if it times out (will dispatch if this flag not set)
        BOOL    startofday:1;       // Set true to send start of day event at startup
        BOOL	shuttles:1;         // Hard coded shuttles on
    } ;
    BYTE	flags;
} CancmdUserFlags;

typedef	union
{
    struct
    {
        BOOL	j7ctrl:1;	// J7 controls which output is main (no effect on BC1a or CANCSB)
        BOOL	mainonboard:1;	// Main output is onboard booster (if J7 disabled)
        BOOL	mainancurrent:1; // Main output has analogue current detection (Set if BC1a has analogue current mod - set true for cancmd and cancsb)
        BOOL	ZTCprog:1;	// Set to use ZTC timing on programming track
        BOOL    trkoffwithstopall:1;    // Turn off track power when stop all received
        BOOL    enableTotiInputs:1;     // Enable track detector inputs on spare inputs
        BOOL    enableRailcomCutout:1;  // Enable Railcom cutout during preamble
        BOOL    enableCutoutInput:1;    // Enable cutout whilst input active (overides enableTotiInputs for use of one input)
    } ;
    BYTE	flags;
} CancmdOpFlags;

typedef	union
{
    struct
    {
        BOOL	prioritypackets:1;	// Send CBUS events for priority DCC packets
        BOOL	refreshspeedpackets:1;	// Send CBUS events for refresh speed packets
        BOOL    refreshfuncpackets:1;   // Send CBUS events for refresh function packets
        BOOL	servicepackets:1;	// Send CBUS events for service mode packets
        BOOL    accessorypackets:1;     // Send CBUS events for accessory control packets
    } ;
    BYTE	flags;
} CancmdDbugFlags;

typedef	struct
{
    BYTE                csnum;                   // To allow for multiple command stations in the future
    CancmdUserFlags     userflags;
    CancmdOpFlags       opflags;
    CancmdDbugFlags     debugflags;
    BYTE		walktimeout;		// Timeout for walkabout (seconds)
    BYTE		maincurrentlimit;	// Raw A/D value for main output current limit
    BYTE		svccurrentlimit;	// Raw A/D value for prog track current limit
    BYTE		currentmult;		// Current multiplier for reporting to give mA
    BYTE		ackdiff;		// Current increment for ACK pulses
    BYTE		shootthrudelay;		// Shoot through delay factor for FET main output (BC1a only)
    WORD		mappednode;		// Node id to recognise for mapped accessory commands - zero to use short events
    BYTE        sendcurrentinterval;    // Time in seconds between sending current events (set to 0 to disable)
    BYTE		sodDelay;		// Delay before sending start of day event (after initial startup delay)
    BYTE        honkInterval;    // How often honk/whistle sounds in POC shuttle
    BYTE		maxSpeed;         // Maximum speed regardless of commands from cab (kids mode!)
    ShuttleEntry	shuttletable[MAX_HANDLES];
} ModuleNodeDefs;		

typedef union
{
    NodeBytes   	nodevars[NV_NUM];
    ModuleNodeDefs	module_nodevars;
} NodevarTable;                                 //  Node variable tabe definition with option to access as an array of bytes


typedef rom ModuleNodeDefs  *ModNVPtr;          //. Pointer type to NV defnitions with cancmd specific meanings

//***************************************************************************************
// Data structures for Event table
// ****************************************************************************************

typedef	union
{
    struct
    {
        unsigned    evtype:2;
        BOOL        onevent:1;
        BOOL        offevent:1;
        BOOL        polarity:1;
        BOOL        toggle:1;
        BOOL        usemain:1;
        BOOL        extended:1;
    } ;
    BYTE	flags;
} EventFlags;


typedef struct
{
    EventFlags  eventflags;
    BYTE        delay;
} CtrlEvent;

typedef	struct
{
    EventFlags  eventflags;     // EV1
    BYTE	delay;		// EV2 Delay before action
    WORD	DCCaddress;	// EV3 DCC accessory address  		\_	If DCC adress is a loco number
    WORD	accnum;		// EV4 DCC accessory number on decoder	/	then send function "accnum" addressed to that loco
} ACCEvent;
		
typedef	struct
{
    EventFlags	eventflags;     // EV1
    BYTE	delay;		// EV2 delay before action 0-256 secs
    BYTE	shuttlenum;	// EV3Index to shuttle table
    unsigned	inputnum:4;	// EV4 F=CBUS event, 0-7 means use spare input pins for detector
    unsigned	operation:4;    // EV4 see enum for operation codes
    BYTE	speed;		// EV5 new speed (and direction), 0xFF for no change
    BYTE	dwell;		// EV6 dwell time  0-256 secs
    BYTE	random;		// EV7 256=always, 0=never
    BYTE	func;		// EV8 action number to send
    BYTE        seqcount;       // EV9 Sequencing counter
    BYTE        spare;           // EV10 spare
} SHTEvent;

typedef	struct
{
    EventEntry  event;          // Event id and hash table stuff, defined in FLiM module
    
    union                       // Cmd station specific EVs
    {   
        CtrlEvent       ctlev;  // EVs for control events
        ACCEvent	accev;  // EVs for accessory events
        SHTEvent	shtev;  // EVs for shuttle events
    } ev;
} CMDEventEntry;

typedef rom CMDEventEntry *ModEVPtr;



// List for outstanding delayed events - unused entries have event index set to 0xFF

enum    eventActions
{
    eaStop = 0,
    eaStart,
    eaHonk,
    eaHonkEnd,
    eaIgnoreButton,
    eaOnEvent,
    eaOffEvent,
};

typedef struct 
{
    BYTE                    delayedEventIndex;  // Index for delayed event (meaning depends on action) 0xFF for entry not in use
    BYTE                    delayCount;
    enum eventActions       action;
    BYTE                    param;              // Parameter, meaning dependent on action
} DelayListEntry ;


enum eventtypes                 // Indicates usage of this event in the event table
{
    etctrl=0,		// General control operations
    etacc,		// DCC accessory control
    etshuttle		// Shuttle control
};

enum ctrlops
{
    opreset=0,          // Reset command station
    optrkpwr,           // Track power on/off
    opstopall           // Stop all
};

enum shuttleops 
{
    shnop=0,		// do nothing
    shrun,		// Set this shuttle running (loco on the stack)
    shhalt,		// Halt this shuttle (release loco)
    shstart,		// Start train moving
    shstop,		// Stop train
    shpause,		// Pause (eg: station stop)
    shreverse,		// Reverse direction
    shforwards,		// Set direction forwards
    shbackwards,        // Set direction backwards
    shfunc,		// Send function (use EV func or overide if event + 1 data byte)
    shspeed,		// Set new speed and direction
    shloco,		// Set loco (teach this using an event + 1 data byte
    shcounter           // Set count value FFFE = icrement, FFFF = decrement or ACON1 is value to set
};

extern const rom NodevarTable	nodevartable;
extern const rom BYTE topofFLiM;

// extern const rom CMDEventEntry	eventtable[MAX_EVT];  // events will be supported in version 5
extern           ModNVPtr       cmdNVptr;                // Pointer to node variables structure

extern ActiveShuttleEntry	activeShuttleTable[ MAX_HANDLES ];
extern DelayListEntry       delayedEvents[ MAX_DELAYED_EVENTS ];


void	cmdFlimInit( void );
void    sendStartOfDay( ModNVPtr cmdNVPtr );
void 	FLiMSWCheck( void );
BYTE 	BlinkLED( BOOL blinkstatus );
void    amperage_messages(void);
void    send_stat( void );
void 	set_shuttle_loco( BYTE session, BYTE shuttle_id );
BOOL    populate_shuttle( BYTE session, BYTE shuttle_id, BOOL ifempty );
BYTE    find_shuttle_session( BYTE session );
void    clear_shuttle_entry( BYTE session );
void    processDelayedEvents(void);
BOOL    addDelayedEvent ( BYTE eventIndex, BYTE delayCount, enum eventActions Action, BYTE param );

#endif	// __cmdFLiM_H

