#ifndef __SHUTTLES_H

//
// MERG CBUS DCC Command Station/Programmer - declarations for shuttle operations

//      All source code is copyright (C) the author(s) concerned
//	Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
//      Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
//      Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2012 Pete Brownlow  merg@upsys.co.uk
//
/*
 *   This work is licensed under the:
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

// Temp hard coded stuff for shuttle proof of concept testing
// The plan is that eventually it will be possible to teach these to CANCMD as consumer events.


#if DRShuttle

#define SH_POC_EN_NODE  115             // Node 
#define	SH_POC_ENABLE_EN 6              //   and event to enable shuttle

#define SH_FWD_NODE     115             // Node
#define SH_FWD_EN       1               //     and event base for forward end reversing sensor 

#define SH_REV_NODE     116             // Node
#define SH_REV_EN       6               //     and event base for reverse end reversing sensor)

#define SH_BUT_NODE     120             // Node for push buttons
#define SH_BUT_EN       6               // Event base for shuttle buttons
#define SH_HONK_EN      9               // 4th button not currently fitted - defined for future use 

#define POC_MAX         2

#else  // Other shuttles using a range of events, base event number plus shuttle no. to define event to use

#define SH_POC_CTL_NODE 162             // Shuttle Control Node 
#define	SH_POC_ENABLE_EN 5              //   event to enable shuttles
#define	SH_POC_START_EN 6              //   event to start shuttles
#define	SH_POC_STOP_EN 7              //   event to stop shuttles

#ifdef KMRS

// Hard code events for Kingsway (KMRS) exhibition until sensor moduels updated to CANMIO (Universal) so producer events can be taught

#define SH_FWD_NODE     1306             // Node
#define SH_FWD_EN       9               //   and event for forward end reversing sensor base event number (shuttle 0)

#define SH_REV_NODE     1311             // Node
#define SH_REV_EN       10              //   and event for reverse end reversing sensor base event number (shuttle 0)
 
#else

#define SH_FWD_NODE     163             // Node
#define SH_FWD_EN       12               //   and event for forward end reversing sensor base event number (shuttle 0)

#define SH_REV_NODE     164             // Node
#define SH_REV_EN       12              //   and event for reverse end reversing sensor base event number (shuttle 0)

#define SH_LKOUT_NODE   165             // Node
#define SH_LKOUT_EN     12              //   and event for shuttle lockout base eventnumber (shuttle 0)

#endif

#define SH_PAUSE_TIME   60              // Delayed event count for shuttle pause at each end
#define SH_BUT_NODE     120             // Node for push buttons
#define SH_BUT_EN       6               // Event base for shuttle buttons
#define SH_HONK_EN      5               // Event for honk button

#define POC_MAX         6              // Maximum "dispatch into" shuttle no.

#define TOTI_DEBOUNCE   10              // Counts round main loop for train detector input debounce

#endif


#endif


// Shuttle tables are stored in NVs, so data structures are defined in CMDFLiM.h

void checkToTiInputs();
void cbus_event(ecan_rx_buffer * rx_ptr, ModNVPtr cmdNVptr);
void setShuttleLockoutState( BYTE shuttleIndex, BYTE opcode );
void requestSensorStates( BYTE shuttleNum );
void reverseShuttleAtSensor( BYTE shuttleIndex, BOOL fwdSenSH_POCsor );
void processDelayedEvent( DelayListEntry eventEntry, ModNVPtr cmdNVPtr );
void setShuttlesAuto( void );
BYTE getShuttleSession( BYTE shuttleIndex );
void stopShuttle( BYTE shuttleIndex);
void reverseShuttle( BYTE shuttleIndex );
void doHonk(BYTE session, BYTE honkTypeCount);
void initShuttles(ModNVPtr cmdNVPtr);
void startShuttles(BOOL reStart);
void stopShuttles(void);
void sendShuttleStatus( BYTE shuttleEvent, BYTE i);
void setShuttleNVs( BYTE shuttleIndex );

#define __SHUTTLES_H
#endif
