/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 MERG CBUS DCC Command Station/Programmer - Shuttle operations

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
	
 For version number and revision history see CANCMD.c
 
*/

#include "project.h"
//#include "commands.h"
//#include "packet_gen.h"
//#include "shuttles.h"

//
// 7/9/24   Pete Brownlow - Refactored shuttle related code into separate source file 


BOOL sh_poc_enabled = FALSE;
// BYTE honkCount;
BYTE honkTypeCount;

#pragma code APP

// Predefined Shuttle tables are stored in NVs, so data structures are defined in CMDFLiM.h

// Shuttle operation from direct digital inputs from train detectors

TOTIEntry Totis[2];  // manage the TOTI inputs

void checkToTiInputs()

{
    BYTE i;
    BOOL totiActive;

    if (cmdNVptr->opflags.enableTotiInputs)
        for (i=0;i<2;i++)
        {
            totiActive = !(i == 0 ? TOTI1 : TOTI2 ); // Get train detector input (active low)         
            
            if (totiActive)
            {    
                if (Totis[i].activeDebounceCount > 0)
                {
                    if (--Totis[i].activeDebounceCount == 0)
                        reverseShuttleAtSensor( 0, (i==0) );  // Direct train detector inputs work on shuttle 0,  first detector forward, second reverse
                }
                else
                    Totis[i].activeDebounceCount = TOTI_DEBOUNCE;
            }
            else
                Totis[i].activeDebounceCount = 0;
                  
        }    
}

// CBUS events for shuttle actions and DCC accessory operations
// Proof of concept shuttle events are currently hard coded

void cbus_event(ecan_rx_buffer * rx_ptr, ModNVPtr cmdNVPtr)
{
    BYTE base_index, shuttle_index, session, opcode;
    WORD eventNode, eventNum;

    eventNode  = rx_ptr->d1;
    eventNode <<= 8;
    eventNode += rx_ptr->d2;    
    
   //  eventNode = (rx_ptr->d1 << 8) + rx_ptr->d2;
    
    eventNum  = rx_ptr->d3;
    eventNum <<= 8;
    eventNum += rx_ptr->d4;
    
    opcode = rx_ptr->d0;
    
    if ((opcode == OPC_ASON) || (opcode == OPC_ASOF) || (opcode == OPC_ARSON) || (opcode == OPC_ARSOF))
        eventNode = 0; // Flag node as zero for short events
    
    // eventNum  = (rx_ptr->d3 << 8) + rx_ptr->d4;  // gets wrong answer compared to 3 lines above - probably issue with promoting byte operand to word
   
    // Mapped CBUS event to DCC accessory - note that CBUS events count from 1 whilst DCC accessory addresses count from zero, so use CBUS event - 1

    if (cmdNVPtr->userflags.mapdccacc) 
    { // if Map CBUS event to DCC accessory command turned on
        if (((opcode == OPC_ASON) || (opcode == OPC_ASOF)) && (cmdNVPtr->mappednode == 0) // Short event
        ||  ((opcode == OPC_ACON) || (opcode == OPC_ACOF)) && (cmdNVPtr->mappednode == eventNode)) // Long event     
        {
            // Send a DCC accessory packet corresponding to the CBUS event received, or a route of packets if found
            if (!dccAccessoryRoute(eventNum, (opcode == OPC_ACON) || (opcode == OPC_ASON)))
                dccAccessoryWrite(eventNum-1, (opcode == OPC_ACON) || (opcode == OPC_ASON));
        }
    }

   // CS Management events
#ifdef KIMBLE_ES
    if ((eventNode == KI_CS_NODE) && (eventNum == KI_STOP_ALL) && (opcode == OPC_ACON))
        stopAll();
#endif
    
    if (eventNode == HC_CS_NODE) 
    {
        switch (eventNum)
        {
            case HC_STOP_ALL:
                if (opcode == OPC_ACON)
                    stopAll();
                break;
                        
            case HC_PWR_CTL:
                power_control(opcode == OPC_ACON);
                break;
                
            case HC_RESET:
                if (cmdNVPtr->userflags.PermitEventReset)
                    Reset();
                break;
        }        
    }    

    
   // Shuttle events

    if (eventNode == SH_POC_CTL_NODE)
    {        
        switch (eventNum)
        {
            case SH_POC_ENABLE_EN:
                sh_poc_enabled = (cmdNVPtr->userflags.shuttles && (opcode == OPC_ACON)); // Check if shuttles enabled (by switch and NV flag)
                setShuttlesAuto();
                break;
                        
            case SH_POC_START_EN:
                startShuttles(TRUE);  // True for restart, so don't need autostart flag set
                break;
                
            case SH_POC_STOP_EN:
                stopShuttles();
                break;
        }        
    }            

    if (sh_poc_enabled) 
    {
        if (eventNode == SH_BUT_NODE)
        {    
            if ((eventNum == SH_HONK_EN) && (opcode == OPC_ACON)) 
            {
                if ((session = getShuttleSession(0)) != 0xFF) {
                    if ((honkTypeCount > 4) || (honkTypeCount < 3))
                        honkTypeCount = 3;

                    doHonk(session, honkTypeCount);

                }
            }

            if ((session = getShuttleSession(shuttle_index = eventNum - SH_BUT_EN)) != 0xFF) 
            {
                if (opcode == OPC_ACON) 
                {
                    activeShuttleTable[ shuttle_index ].flags.manual = TRUE;
                    if ((q_queue[session].speed & 0x7F) == 0) 
                    {
                        speed_update(session, activeShuttleTable[ shuttle_index ].set_speed);

                        if ((honkTypeCount > 4) || (honkTypeCount < 3))
                            honkTypeCount = 3;

                        doHonk(session, honkTypeCount);

                        honkTypeCount++;
                    }
                }

                if ((opcode == OPC_ACOF) && (q_queue[session].speed & 0x7F) != 0) {
                    if (activeShuttleTable[ shuttle_index ].flags.manual) {
                        activeShuttleTable[ shuttle_index ].set_speed = q_queue[session].speed;
                        speed_update(session, activeShuttleTable[ shuttle_index ].set_speed & 0x80); // set speed to zero but leave direction intact
                    }
                }
            }
        }
 
        if (((eventNode == SH_FWD_NODE) || (eventNode == SH_REV_NODE)) && ((opcode == OPC_ACON) || (opcode == OPC_ARON) || (opcode == OPC_ASON) || (opcode == OPC_ARSON) ))
        {    
            base_index = (eventNode == SH_FWD_NODE ? SH_FWD_EN : SH_REV_EN );
            
            shuttle_index = eventNum - base_index;
            sendShuttleStatus(SHUTTLE_EVENT_SENSOR, (eventNode == SH_FWD_NODE));
            reverseShuttleAtSensor( shuttle_index, (eventNode == SH_FWD_NODE) );
            
        } // if reversing sensor node number
        
        if (eventNode == SH_LKOUT_NODE)
        {
            shuttle_index = eventNum + SH_LKOUT_EN;
            setShuttleLockoutState( shuttle_index, opcode);
        }            

    } //  if POC shuttles enabled
} // cbus_event


void setShuttleLockoutState( BYTE shuttleIndex, BYTE opcode )
{
    if (activeShuttleTable[shuttleIndex].flags.valid)
    {
        if (activeShuttleTable[shuttleIndex].flags.initialisedorLockedOut = (opcode == OPC_ACON))
            stopShuttle( shuttleIndex );                                     // stop shuttle if locked out
        else
            addDelayedEvent(shuttleIndex, SH_PAUSE_TIME, eaStart,0 );         // restart after a delay if lockout cleared
            
    }    
}



void requestSensorStates( BYTE shuttleNum )

{
    WORD    fwdEvent, revEvent, lckEvent;
    
    fwdEvent = SH_FWD_EN + shuttleNum;
    revEvent = SH_REV_EN + shuttleNum;
    lckEvent = SH_LKOUT_EN + shuttleNum;
    
    Tx1[d0] = (SH_FWD_NODE == 0 ? OPC_ASRQ : OPC_AREQ);
    Tx1[d3] = fwdEvent >> 8;
    Tx1[d4] = fwdEvent & 0xFF;
    sendCbusMsgNN(SH_FWD_NODE);
    
    Tx1[d0] = (SH_REV_NODE == 0 ? OPC_ASRQ : OPC_AREQ);
    Tx1[d3] = revEvent >> 8;
    Tx1[d4] = revEvent & 0xFF;
    sendCbusMsgNN(SH_REV_NODE);
    
    Tx1[d0] = (SH_LKOUT_NODE == 0 ? OPC_ASRQ : OPC_AREQ);
    Tx1[d3] = lckEvent >> 8;
    Tx1[d4] = lckEvent & 0xFF;
    sendCbusMsgNN(SH_LKOUT_NODE);
        
    // Responses will be handled in cbus_event    
}

void reverseShuttleAtSensor( BYTE shuttleIndex, BOOL fwdSensor )

{
    
    BYTE        session;
    DCCSpeed    setSpeed;
    BOOL        doReverse, fwdDir;
    
    session = getShuttleSession(shuttleIndex);
                
    if ( session != 0xFF) 
    {
        setSpeed.velocity = activeShuttleTable[shuttleIndex].set_speed;
        sendShuttleStatus(SHUTTLE_EVENT_FLAGS, activeShuttleTable[shuttleIndex].flags.byte);
        sendShuttleStatus(SHUTTLE_EVENT_SPEED, setSpeed.velocity);        

        if (!activeShuttleTable[shuttleIndex].flags.directionSet)
        {
            activeShuttleTable[shuttleIndex].flags.fwdDirBit = (fwdSensor && !(setSpeed.direction));  // Flag to say that forward direction uses the forward sensor
            activeShuttleTable[shuttleIndex].flags.directionSet = TRUE;
            sendShuttleStatus(SHUTTLE_EVENT_DIRSET, activeShuttleTable[shuttleIndex].flags.byte);
        }    

        if ((activeShuttleTable[shuttleIndex].flags.fwdDirBit == setSpeed.direction)  == !fwdSensor)
            reverseShuttle(shuttleIndex);
    }
}

// For POC, the delayed event is always from turning off honk/whistle or restarting a shuttle after pause

void processDelayedEvent(DelayListEntry eventEntry, ModNVPtr cmdNVPtr)
 {
    BYTE eventIndex, session;
    int i;

    eventIndex = eventEntry.delayedEventIndex;
    
    switch (eventEntry.action)
    {
        case eaHonk:

            if ((session = getShuttleSession(eventIndex)) != 0xFF)
            {
                if (--(activeShuttleTable[eventIndex].counter) == 0)  // Check if it is time to whistle/honk
                {
                    if ((honkTypeCount > 4) || (honkTypeCount < 3))
                        honkTypeCount = 3;

                    doHonk(session, honkTypeCount++);
                    activeShuttleTable[eventIndex].counter = cmdNVPtr->honkInterval;

                }        
            }
            break;
            
        case eaHonkEnd:
            if ((session = getShuttleSession(eventIndex)) != 0xFF)
                loco_function(funcoff, session, eventEntry.param); // end honk/whistle
            break;
        

        case eaStart:
            if (((session = getShuttleSession(eventIndex)) != 0xFF) && (!activeShuttleTable[ eventIndex ].flags.paused) && (!activeShuttleTable[ eventIndex ].flags.initialisedorLockedOut))
            {    
                speed_update(session, activeShuttleTable[ eventIndex ].set_speed); // set speed to stored value
                sendShuttleStatus(SHUTTLE_EVENT_RESUME,activeShuttleTable[ eventIndex ].set_speed);        
            }    
            if (!activeShuttleTable[ eventIndex ].flags.initialisedorLockedOut)
                addDelayedEvent(eventIndex, SH_PAUSE_TIME, eaStart ,0); // Try to start again after a delay
            
            break;            
            
        case eaRqSensors:
            if ((session = getShuttleSession(eventIndex)) != 0xFF)
            {
                requestSensorStates( eventIndex );
            }    
                
            
    } // switch
    
}

void setShuttlesAuto(void)
 {
    BYTE i;

    for (i = 0; i < MAX_SHUTTLES; i++) {
        if (activeShuttleTable[i].flags.valid)
            activeShuttleTable[i].flags.manual = FALSE;
    }
} // setShuttlesAuto

BYTE getShuttleSession(BYTE shuttleIndex) // Return 0xFF if shuttle index or session not active
 {
    BYTE session = 0xFF;

    if ((shuttleIndex <= POC_MAX) && (activeShuttleTable[ shuttleIndex ].flags.valid)) {
        session = activeShuttleTable[ shuttleIndex ].session;
        
        if (q_queue[session].status.valid != 1)
            session = 0xFF;
        
        sendShuttleStatus(SHUTTLE_EVENT_SESSION,session);        
    }
    return ( session);
} // getShuttleSession

void reverseShuttle(BYTE shuttleIndex)
 {
    BYTE session;
    BYTE newSpeed;

    if ((session = getShuttleSession(shuttleIndex)) != 0xFF) {
        activeShuttleTable[ shuttleIndex ].set_speed ^= 0x80; // toggle direction

        newSpeed = activeShuttleTable[ shuttleIndex ].set_speed;

        if (!activeShuttleTable[ shuttleIndex ].flags.manual) {
            newSpeed &= 0x80; // In automatic mode, set speed to zero but with new direction whilst delay at each end of shuttle
            addDelayedEvent(shuttleIndex, SH_PAUSE_TIME, eaStart ,0); // Set off again after a delay
        }
        // loco_function( funcoff, session, 3 );		// Turn off sound functions
        // loco_function( funcoff, session, 4 );		// Turn off sound functions

        speed_update(session, newSpeed);
        sendShuttleStatus( SHUTTLE_EVENT_REV, activeShuttleTable[ shuttleIndex ].set_speed);
    }
} // reverseShuttle

void stopShuttle( BYTE shuttleIndex )
{
    BYTE session;
    BYTE newSpeed;
    
    if ((session = getShuttleSession(shuttleIndex)) != 0xFF)
    {
        newSpeed = (activeShuttleTable[ shuttleIndex ].set_speed & 0x80);  // set speed to zero but leave direction unchanged
        speed_update(session, newSpeed);
    }
}


void doHonk(BYTE session, BYTE honkTypeCount)    // honk or whistle
{
    int i;

    if ((honkTypeCount > 4) || (honkTypeCount < 3))
        honkTypeCount = 3;
    loco_function(funcon, session, honkTypeCount); // send a honk
    addDelayedEvent( session, 4, eaHonkEnd, honkTypeCount );
}


void initShuttles(ModNVPtr cmdNVPtr)

{
    int i;
    
    sh_poc_enabled = cmdNVPtr->userflags.shuttles;
//#if DEBUG_SHUTTLES || FFQ_SHUTTLES  
//    sh_poc_enabled = TRUE;
//#endif    
    
    
 	// Initialise active shuttle table and copy predefined shuttles info from NVs into active shuttle table

	for (i=0; i<MAX_SHUTTLES; i++)
	{
        activeShuttleTable[i].counter = 0;
        activeShuttleTable[i].session = 0xFF; 

        if (nodevartable.module_nodevars.shuttletable[i].flags.initialisedorLockedOut && nodevartable.module_nodevars.shuttletable[i].flags.valid )
        {    
            activeShuttleTable[i].flags.byte = nodevartable.module_nodevars.shuttletable[i].flags.byte;
            activeShuttleTable[i].loco_addr =  nodevartable.module_nodevars.shuttletable[i].loco_addr;
            activeShuttleTable[i].set_speed =  nodevartable.module_nodevars.shuttletable[i].default_speed;
            activeShuttleTable[i].flags.fwdDirBit = (nodevartable.module_nodevars.shuttletable[i].default_speed & 0x80) == 0;   
            activeShuttleTable[i].flags.directionSet = TRUE;
            activeShuttleTable[i].flags.initialisedorLockedOut = FALSE;
            // Send status event for shuttle found
            // sendShuttleStatus( SHUTTLE_EVENT_INIT, i);
        }
        else
            activeShuttleTable[i].flags.byte = 0;
 	}
  
    // Initialise delayed event table

    for (i=0; i<MAX_DELAYED_EVENTS; i++)
    {
        delayedEvents[i].delayCount = 0;
        delayedEvents[i].delayedEventIndex = 0xFF;
    }   
    
    // Initialise TOTI management
    
    for (i=0;i<2;i++)
    {
        Totis[i].activeDebounceCount = 0;
        Totis[i].inactiveDebounceCount = 0;
    }    
    
}

void startShuttles(BOOL reStart)

{
    BYTE shuttleNum, session;
    
    if (sh_poc_enabled)
    {    
        // Start or restart any predefined shuttles. If we are starting at reset, only those with the autostart flag set
        
        sendCbusEvent( SHUTTLE_EVENT_STARTED, TRUE );
        for (shuttleNum=0; shuttleNum<MAX_SHUTTLES; shuttleNum++)
        {
            if (activeShuttleTable[shuttleNum].flags.valid && (activeShuttleTable[shuttleNum].flags.autostart || reStart))
            {
                // Send status event for shuttle found
                // sendShuttleStatus( SHUTTLE_EVENT_FOUND, shuttleNum);

                // Create a loco session for shuttle if it doesn't already exist, will send a ploc if successful so we can see it did it on CBUS
                if ((session = activeShuttleTable[ shuttleNum ].session ) == 0xFF)
                {    
                    session = queue_add(activeShuttleTable[shuttleNum].loco_addr.addr_int, glocNormal, (ModNVPtr) cmdNVptr); 
                    activeShuttleTable[ shuttleNum ].session = session;
                    activeShuttleTable[ shuttleNum ].flags.started = TRUE;
                    q_queue[session].status.shuttle = 1;
                    // sendShuttleStatus( SHUTTLE_EVENT_SESSION, session);
                }    

                if (session != 0xFF)  
                {    
                    activeShuttleTable[ shuttleNum ].flags.paused = FALSE;
                    
                // Turn on lights and sound (assumed F0 for lights and F1 for sound)
                    loco_function(funcon, session, 0);  // Lights
                    loco_function(funcon, session, 1);  // Sound (or carriage lights on some))                
                
                // Set shuttle going at speed stored in shuttle table    
                  
                    speed_update(session, activeShuttleTable[ shuttleNum ].set_speed);
                    sendShuttleStatus( SHUTTLE_EVENT_SPEED, activeShuttleTable[ shuttleNum ].set_speed);
                    
                // Request status of sensors at each end, so if sensor we are heading for is already active, train will immediately reverse and go the right way when the response is received
                    addDelayedEvent(shuttleNum,shuttleNum+3,eaRqSensors,0);
                    // requestSensorStates( shuttleNum );
                }    
            }  
        } 
    }
    sendShuttleStatus( SHUTTLE_EVENT_STARTED, 0 );
}    

    
void stopShuttles(void)

{
    BYTE shuttleNum, session;
 
    sendCbusEvent( SHUTTLE_EVENT_STARTED, FALSE );
    // Stop all running shuttles 
    for (shuttleNum=0; shuttleNum<MAX_SHUTTLES; shuttleNum++)
    {
        if (activeShuttleTable[shuttleNum].flags.valid)
        {
            // Send status event for shuttle found
            // sendShuttleStatus( SHUTTLE_EVENT_FOUND, shuttleNum);

            // Save current speed and set speed to zero
            session = activeShuttleTable[ shuttleNum ].session;
            activeShuttleTable[ shuttleNum ].flags.paused = TRUE;
 
            
            // activeShuttleTable[ shuttleNum ].set_speed = q_queue[session].speed;
            speed_update(session, 0);
        }  

    }
}    


void sendShuttleStatus( BYTE shuttleEvent, BYTE i)
{
#if DEBUG_SHUTTLES
    
    if (startupTimer == 0)
    {    
        Tx1[d0] = OPC_ACON3;
        Tx1[d3] = 0;
        Tx1[d4] = shuttleEvent;
        Tx1[d5] = i;
    //    Tx1[d5] = activeShuttleTable[i].set_speed;
        Tx1[d6] = activeShuttleTable[i].loco_addr.addr_hi.byte;
        Tx1[d7] = activeShuttleTable[i].loco_addr.addr_lo;
        sendCbusMsgNN(Node_id);      
    }
#endif    
}

void setShuttleNVs( BYTE shuttleIndex )
{
    // This is used to set the NVs for a shuttle in response to the user setting a loco into a shuttle from the handset
 
    BYTE            NVindx;
    ShuttleFlags    sflags;
     
    NVindx = SHUTTLE_TABLE_NV;
    
    NVindx += (shuttleIndex * sizeof(ShuttleEntry));
    
    doNvset(NVindx++, activeShuttleTable[shuttleIndex].loco_addr.addr_lo);    
    doNvset(NVindx++, activeShuttleTable[shuttleIndex].loco_addr.addr_hi.byte);
    doNvset(NVindx++, activeShuttleTable[shuttleIndex].set_speed);
    sflags = activeShuttleTable[shuttleIndex].flags;
    sflags.autostart = TRUE;
    sflags.started = FALSE;                 // In NVs table shuttle is not yet started as this is used at startup
    doNvset(NVindx++, sflags.byte);    

}