/*
        Code based on Mike Bolton's canacc8 for which due credit is given. 
        Modifications and conversion to C18 Copyright (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 Microchip C18 source for common CBUS routines

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
	
 * Changed for 8 MHz resonator to double the clock. Mike Bolton  19/04/11
 *
 * Rationalised to separate CAN and CBUS into separate source modules and added self enumeration. Pete Brownlow Nov 12

 For version number and full revision history see CANCMD.c
 
*/ 


#include <GenericTypeDefs.h>
#include "cbus.h"
#include "project.h"

#pragma code APP



// Initialise CBUS

void initCbus( BYTE EEcanID, BYTE EEnodeID )
{
    initCan(EEcanID);
}


/*
 * Send single byte frame - opcode only
 */
void sendCbusOpc(BYTE opc) {
	Tx1[d0] = opc;
	sendCbusMsg();
}


/*
 * Send simple 3-byte frame, opcode plus node number
 */
void sendCbusOpcNN(BYTE opc, WORD Node_id) {
	Tx1[d0] = opc;
	sendCbusMsgNN(Node_id);
}

/*
 * Send a CBUS message, putting Node Number in first two data bytes
 */
void sendCbusMsgNN(WORD Node_id) {
	Tx1[d1] = Node_id>>8;
	Tx1[d2] = Node_id&0xFF;
	sendCbusMsg();
}

//
// Send a Bus On message
//
void setCbusOn(void) {
   sendCbusOpc(OPC_BON);
}

/*
 * Send a CAN frame where data bytes have already been loaded
 * Works out data length from opcode
 */
void sendCbusMsg(void) {
	Tx1[dlc] = (Tx1[d0] >> 5)+1;		// data length
	sendTX1();			
}

void sendCbusEvent( WORD eventNum, BOOL onEvent )
{
    Tx1[d3] = eventNum >> 8;
    Tx1[d4] = eventNum & 0xFF;
    sendCbusOpcNN( (onEvent? OPC_ACON : OPC_ACOF ), Node_id );        
}


/*
 * Send a debug message with 5 data bytes
 */
void sendCbusDebugEvent(WORD Node_id, BYTE *debug_data )
{

    Tx1[d0] = OPC_ACDAT;
    Tx1[d3] = debug_data[0];
    Tx1[d4] = debug_data[1];
    Tx1[d5] = debug_data[2];
    Tx1[d6] = debug_data[3];
    Tx1[d7] = debug_data[4];
    sendCbusMsgNN(Node_id);
}

		


