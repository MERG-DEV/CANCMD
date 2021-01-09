/*

(C) 2011-2017 Pete Brownlow  merg@upsys.co.uk
 
	Generic CBUS FLiM library routines

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


#include "FLiM.h"
// #include "cmdFLiM.h"
// #include "cancmd.h"

// Access device id in config memory

#pragma romdata CPUID
WORD    deviceid;      // Device id in config memory
#pragma romdata



// Static variables local to this library

#pragma udata MAIN_VARS

WORD Node_id;       		//	Copy from EEPROM
enum FLiMStates flimState;      //      This is stored in EEPROM with node id
enum FLiMStates prevFlimState;  //      Store previous state in setup mode
	

rom	NodeBytes 	*NVPtr;
// rom EventTableEntry *EVTPtr;

   
#pragma code APP
#pragma udata

// FLimInit called during intialisation 

void	FLiMinit(void)

{
       // get node id and flim mode from EEPROM

        Node_id = ee_read_short( EE_Node_id);
        flimState = ee_read(EE_FlimMode);
        initRomOps();
 

	// Check validity of event has table, initialise if not
}


// Enter FLiM setup mode - called when module wants to be allocated a node number for FLiM
//	(for example, when pusbbutton pressed.) 
// Returns the node number allocated

void	FLiMsetup( void )

{
    prevFlimState = flimState;
    flimState = fsFLiMSetup;
    // ??? canid self enumeration
    // send request node number packet
    sendCbusOpcNN(OPC_RQNN, Node_id);
   
} // FLiMsetup


// Cancel out of FLiM setup mode 

void FLiMEndSetup( void )

{
    flimState = ((prevFlimState = fsPressedFLiM) ? fsFLiM : fsSLiM );
}


// Sends node number to refresh and goes into setup mode??

void FLiMrefresh()

{
	// ??? should also redo canid self enumeration?
}

// Revert to SLiM mode - called when module wants to go back to slim
// for example when pushbutton pressed whilst in FLiM mode


void SLiMrevert(void)

{
    flimState = fsSLiM;
	// ?? send nn release packet
} // SLiMrevert



// Process CBUS opcode for FLiM - called after any module specific CBUS opcodes have 
// been dealt with.  Returns true if the opcode was processed.
//
BOOL parse_FLiM_cmd(void)
{
    overlay BOOL     cmd_processed = FALSE;

    if (rx_ptr->d0 == OPC_QNN)
    {
        QNNrespond();          // Respond to node query 	//  update to do new spec response agreed
  	
	// All other FLiM commands (except those in setup mode) 
	//	are addressed to a particular node, so we only need
	// to process the command if it is for us

    } else if (thisNN())
    {
        cmd_processed = TRUE;

    	switch(rx_ptr->d0) 
        {
            case OPC_RQNPN:
                    // Read one node parameter by index
                doRqnpn(rx_ptr->d3);
                break;

            case OPC_NNLRN:
                // Put node into learn mode
                if (flimState == fsFLiM)
                    flimState = fsFLiMLearn;
                break;

            case OPC_NNULN:
                // Release node from learn mode
                 flimState = fsFLiM;
                break;

            case OPC_NNCLR:
                // Clear all events
                doNnclr;
                break;

            case OPC_NNEVN:
                // Read available event slots
                doNnevn();
                break;

            case OPC_NERD:
                // Read all stored events
                doNerd();
                break;

            case OPC_RQEVN:
                // Read number of stored events
                doRqevn();
                break;

            case OPC_NVRD:
                // Read value of a node variable
                doNvrd(rx_ptr->d3);
                break;

            case OPC_EVULN:
                // Unlearn event
                doEvuln();
                break;

            case OPC_NVSET:
                // Set a node variable
                doNvset(rx_ptr->d3, rx_ptr->d4);
                break;

            case OPC_REVAL:
                // Read event variable by index
                doReval();
                break;

            case OPC_REQEV:
                // Read event variable by event id
                doReqev();
                break;

            case OPC_EVLRN:
                // Teach event whilst in learn mode
                doEvlrn;
                break;

            case OPC_EVLRNI:
                // Teach event whilst in learn mode with event index
                doEvlrni;
                break;

            case OPC_NNRST:
                // Reset node
                Reset();
                
            default:
                    cmd_processed = FALSE;
            break;
		}
	}

	// In setup mode, also check for FLiM commands not addressed to 
	// any particular node

	if	((!cmd_processed) && (flimState == fsFLiMSetup))
	{
		cmd_processed = TRUE;

		switch(rx_ptr->d0) 
		{
			case OPC_RQNP:	
				// Read node parameters
				doRqnp();
				break;

                        case OPC_RQMN:
                                // Read module type name
                                doRqmn();
                                break;

			case OPC_SNN:
				// Set node number
				doSnn();     
				break;


			default: 
				cmd_processed = FALSE;
        		break;              
		}
	}

	return( cmd_processed );
} // parse_FLiM_cmd


// Internal functions


// QNN Respond

// Send response bytes to QNN query

void QNNrespond()

{
    FLiMprmptr  paramptr;

    paramptr = (FLiMprmptr)&FLiMparams;

    Tx1[d0] = OPC_PNN;
    Tx1[d3] = paramptr->params.manufacturer;
    Tx1[d4] = paramptr->params.module_id;
    Tx1[d5] = paramptr->params.module_flags;
    sendCbusMsgNN(Node_id);
}

// Send node number ackowledge

void doNNack( void )
{
	sendCbusOpcNN(OPC_NNACK, Node_id);
} // doNNack

// Read one node parameter by index
void doRqnpn(BYTE idx)
{
    FLiMprmptr  paramptr;

    paramptr = (FLiMprmptr)&FLiMparams;

    if (idx <= FCUparams.parameter_count)
    {
        Tx1[d0] = OPC_PARAN;
	Tx1[d3] = idx;

        if (idx == 0)
            Tx1[d4] = FCUparams.parameter_count;
        else if ((idx >= PAR_CPUMID) && (idx < PAR_CPUMAN)  )
            Tx1[d4] = readCPUType() >> (( idx - PAR_CPUMID )*8);
        else
            Tx1[d4] = paramptr->bytes[idx-1];

        if ((idx == PAR_FLAGS) && (flimState == fsFLiM))
            Tx1[d4] |= PF_FLiM;

	sendCbusMsgNN(Node_id);
    }
    else
    {
        doError(CMDERR_INV_PARAM_IDX);
    }
} // doRqnpn


// Clear all events
void doNnclr(void)
{
	if (flimState == fsFLiMLearn)
	{
		// ?? clear event table
	}
	else
	{
		// ?? do we need to issue some sort of nack?
	}
} // doNnclr

// Read number of available event slots
void doNnevn(void)
{
	// ?? send response 
} // doNnevn

// Read all stored events
void doNerd(void)
{
	// ?? send response OPC_ENRSP (presumably once for each event)
} // doNerd

// Read number of stored events
void doRqevn(void)
{
	// ?? send response 
} // doRqevn

// Read a node variable
void doNvrd(BYTE NVindex)
{
    // Get NV indexr and send response with value of NV
    Tx1[d0] = OPC_NVANS;
    Tx1[d3] = NVindex;
    Tx1[d4] = *NVPtr[--NVindex];
    sendCbusMsgNN( Node_id );
} // doNvrd

// Unlearn event
void doEvuln(void)
{
	// Find event in event table (may be more than one entry)
	// and remove
} // doEvuln

// Set a node variable
void doNvset(BYTE NVindex, BYTE NVvalue)
{
    WORD flashIndex;

   // *NVPtr[--NVindex] = NVvalue; // Set value of node variable (NV counts from 1 in opcode, adjust index to count from zero)

    flashIndex = (WORD)NVPtr;
    flashIndex += NVindex;
    flashIndex--;

    writeFlashByte(flashIndex, NVvalue);

//    writeFlashByte((WORD)NVPtr+NVindex-1, NVvalue);
    sendCbusOpcNN(OPC_WRACK, Node_id);
       
} // doNvset


// Read an event variable by index
void doReval(void)
{
	// Get event index and event variable number from message
	// Send response with EV value
} // doReval


// Read an event variable by event id
void doReqev(void)
{
	// Get event by event id and event variable number from message
	// Send response with EV value, one message for each EV
	// spec says this is for learn mode, but I see no harm in responding to 
	// this request whether in learn mode or not.
} // doReqev


// Teach event whilst in learn mode
void doEvlrn(void)
{
	// Need to be in learn mode so we know what this event will do when taught
	// Get event id, ev index and ev value from message, store in NV 
    // ?? send ack response
} // doEvlrn


// Teach event whilst in learn mode with event indexing
void doEvlrni(void)
{
	// Need to be in learn mode - index defines what this event will do
	// Get event id, event index, ev index and ev value from message, store in NV 
    // ?? send ack response
} // doEvlrni


// Read node parameters in setup mode
// Returns the first set of parameters that will fit in a CAN message
// Additional parameters added since this opcode was defined can only be read using RQNPN

void doRqnp(void)
{
    FLiMprmptr  paramptr;
    BYTE        copy_counter;

    paramptr = (FLiMprmptr)&FLiMparams;

    Tx1[d0] = OPC_PARAMS;

    for (copy_counter = d1; copy_counter <= d7; copy_counter++)    
    {
        Tx1[copy_counter] = paramptr->bytes[copy_counter-d1];
    }
    if (flimState == fsFLiM)
        Tx1[d1+PAR_FLAGS-1] |= PF_FLiM;

    sendCbusMsg();

} // doRqnp

// Read node parameters in setup mode
// Returns the first set of parameters that will fit in a CAN message
// Additional parameters added since this opcode was defined can only be read using RQNPN

void doRqmn(void)
{
    BYTE        copy_counter;
    rom char*   namptr;
    DWORD       namadr;

    namadr = FCUparams.module_type_name;
    namptr = (rom char*)namadr;

    
    Tx1[d0] = OPC_NAME;

    for (copy_counter = 0; copy_counter++; copy_counter < 7)
    {
        Tx1[copy_counter+d1] = namptr[copy_counter];
    }
    sendCbusMsg();

} // doRqmn


// Set node number while in setup mode
void doSnn(void)
{

    // Get new node number for FLiM
    Node_id = rx_ptr->d1;
    Node_id <<= 8;
    Node_id +=  rx_ptr->d2;
    flimState = fsSetupDone;
    
    // Store new node id and mode to nonvol EEPROM

    SaveNodeDetails(Node_id, fsFLiM);
 
    // Acknowledge new node id

    sendCbusOpcNN( OPC_NNACK, Node_id );
 
} // doSnn



void doError(unsigned int code)
{
		Tx1[d0] = OPC_CMDERR;
		Tx1[d3] = code;
		sendCbusMsgNN(Node_id);
}

BOOL thisNN()
{
	if (((rx_ptr->d0 >> 5) >= 2) && (((unsigned short)(rx_ptr->d1)<<8) + rx_ptr->d2) == Node_id)
	 	return TRUE;
	 else
	 	return FALSE;
}

void SaveNodeDetails(WORD Node_id, enum FLiMStates flimState)

{
    ee_write_short(EE_Node_id, Node_id);
    ee_write(EE_FlimMode, flimState);
} // SaveNodeDetails

WORD readCPUType( void )

{
    WORD    id;

    INTCONbits.GIE = 0;

    id = *(far rom WORD*)0x3FFFFE;

    TBLPTRU = 0;
    INTCONbits.GIE = 1;

    return( id );
}