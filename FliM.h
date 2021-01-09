#ifndef __FLiM_H
#define __FLiM_H

/*

(C) 2011-2017 Pete Brownlow  merg@upsys.co.uk
 
	Definitions for Generic CBUS FLiM library routines

	The FLiM routines have no code or definitions that are specific to any
	module, so they can be used to provide FLiM facilities for any module whose
	firmware is coded in Microchip C18.

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
	
	9/5/11	Pete Brownlow	- Initial outline
	2/6/11	Pete Brownlow	- Define data structures and function prototypes

 For version number and full revision history see CANCMD.c
*/ 


#include "cbus.h"
#include "romops.h"

// EEPROM usage

#define EE_CAN_ID   0
#define EE_Node_id  2   // 16 bit value for node number
#define EE_FlimMode 4   // Flag for FLiM mode


// The parameter bytes can be accessed either as an array of bytes or as a structure with each
// byte name for its meaning, as defined in this union.

// These routines have no knowledge of where the parameter bytes are stored, they are always 
// passed as a parameter.

typedef struct
{
    BYTE	manufacturer;
    BYTE	minor_ver;
    BYTE	module_id;
    BYTE	number_of_events;
    BYTE	evs_per_event;
    BYTE	number_of_nvs;
    BYTE	major_ver;
    BYTE        module_flags;
    BYTE        cpu_id;
    BYTE        bus_type;
    DWORD       load_address;
    DWORD       cpumid;
    BYTE        cpuman;
    BYTE        beta;
} ParamVals;

typedef	union
{
    ParamVals   params;
    BYTE	bytes[sizeof(ParamVals)];
} FLiMParameters;

typedef rom ParamVals       *prmvalptr;
typedef rom FLiMParameters  *FLiMprmptr;


typedef BYTE    SpareParams[4];

typedef struct
{
    WORD        parameter_count;
    DWORD       module_type_name;
    WORD        parameter_checksum;
} FCUParams;

typedef struct
{
    ParamVals   vals;
    SpareParams spares;
    FCUParams   FCUparams;
} ParamBlock;


// NODE VARIABLES
//
// For node variables, the application code will define a union of the form:
//
//	Typedef 
//		union
//		{
//			NodeBytes	nodevars[NUMBER_OF_NVS];
//			ModuleNodeDefs	module_nodevars;
//		} NodevarTable
//
//  Where ModuleNodeDefs is a structure type that defines the meaning and structure
//  of each node variable as used by that application.
//
//  NVPtr is then defined to point to the NodeBytes member of the union, thus giving the
//  generic FLiM code access to the node variables table without requring any knowledge
//  of the structure of the node variables themselves. 


typedef	BYTE		NodeBytes[];


// EVENTS
//
// The events are stored as a hash table in flash (flash is faster to read than EEPROM)
// There can be up to 256 events. Since the address in the hash table will be 16 bits, and the
// address of the whole event table can also be covered by a 16 bit address, there is no
// advantage in having a separate hashed index table pointing to the actual event table.
// Therefore the hashing algoritm prouduces the index into the acual event table, which
// can be shifted to give the address - each event table entry is 16 bytes. After the event
// number and hash table overhead, this allows up to 10 EVs per event.
//
//  For the event table, the application code will define a structure of the form:
//  typedef	struct
//  {
//      EventEntry      event;          // Event id and hash table management, defined in FLiM module
//      ModuleEvs       modEvs;         // Application specific EV definitions
//  } ; ModuleEventEntry;
//
// Where ModuleEvs defines the specific EV usage by that application
// ModuleEvs may itself be a further combination of structures and/or unions depending on the EV usage by that code,
// the generic FLiM code needs no knowledge of specific EV usage
//
//  EVPtr is then defined to point to an EventTable Entry structue, which mirrors the application specific definition without the EV specific usage information.
//  thus giving the generic FLiM code access to the event table, and event hash table information, without requring any knowledge
//  of the structure of the event variables themselves.
//  The application code can use the same pointer, type transferred to its own modEVptr, to access application specific information in the event table.

typedef union
{
    struct
    {
        BOOL    producer:1;     // Set for producer event
    };
    BYTE    free;       // Set to 0xFF for free entry, initially set to zero for entry in use, then producer flag set if required.
} EventTableFlags;

typedef struct
{
    WORD		node_id;
    WORD		evt_id;
    BYTE                nextEvent;     // Event index to Link to next entry on the same hash value. Zero terminated list - linked event cannot be at index 0
    EventTableFlags     evtFlags;      // Set to 0xFF for a free entry
} EventEntry;

typedef
    BYTE                evBytes[10];    // 10 bytes for EVs in each event table entry

typedef struct
{
    EventEntry          event;
    evBytes             ev;
} EventTableEntry;


typedef EventTableEntry evEntry[];

// State machine for FliM operations

enum FLiMStates
{
    fsSLiM=0,		// In SLiM, button not pressed
    fsFLiM,		// In FLiM mode
    fsPressed,          // Button pressed, waiting for long enough
    fsFlashing,         // Button pressed long enough,flashing now
    fsFLiMSetup,        // In FLiM setup mode (LED flashing)
    fsSetupDone,	// Exiting setup mode
    fsFLiMLearn,        // In FLiM learn mode
    fsPressedFLiM,      // Pressed whilst in FLiM mode
    fsPressedSetup      // Pressed during setup
};

// External variables for other modules to access FLiM

extern rom	NodeBytes 	*NVPtr;     // pointer to Node variables table.  \_ These can be array as defined here or with specific structures in module specific code
// extern rom	EventTableEntry	*EVTPtr;    // pointer to Event variables table. /

extern WORD     Node_id;
extern BOOL	NV_changed;	
extern enum     FLiMStates flimState;

// Data stored in program ROM - FLiM parameters. Node Variables and Event Variables storage definition is in the application module.

extern const rom ParamVals  FLiMparams;
extern const rom FCUParams  FCUparams;
extern const rom char       module_type[];


// Function prototypes for FLiM operation

void 	FLiMinit( void );

// Enter FLiM mode - called when module wants to be allocated a node number for FLiM
//	(for example, when pusbbutton pressed.)
// Returns the node number allocated

void	FLiMsetup( void );


// End FLiM setup mode

void FLiMEndSetup( void );

// Sends node number to refresh and goes into setup mode??

void FLiMrefresh( void );

// Revert to SLiM mode - called when module wants to go back to slim
// for example when pushbutton pressed whilst in FLiM mode

void	SLiMrevert(void);

// Process FLiM opcode - called when any module specific CBUS opcodes have 
// been dealt with.  Returns true if the opcode was processed.

BOOL 	parse_FLiM_cmd(void);


// Internal functions

void    QNNrespond( void );
void 	doNNack( void );
void	doRqnpn(BYTE idx);
void 	doNnclr(void);
void 	doNnevn(void);
void 	doNerd(void);
void 	doRqevn(void);
void 	doNvrd(BYTE NVindex);
void	doEvuln(void);
void	doNvset(BYTE NVindex, BYTE NVvalue);
void 	doReval(void);
void 	doReqev(void);
void 	doEvlrn(void);
void 	doEvlrni(void);
void 	doRqnp(void);
void    doRqmn(void);
void 	doSnn(void);
void 	doReval(void);
void	doError(unsigned int code);
BOOL	thisNN(void);
void    SaveNodeDetails(WORD Node_id, enum FLiMStates flimState);
WORD    readCPUType( void );


#endif	// __FLiM_H
