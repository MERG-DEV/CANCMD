
/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 This code is for a CBUS DCC Command Station
 
 MERG CBUS DCC Programmer/Command Station - CBUS command parser
  
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


// 06/04/11 Roger Healey - Modify code for OPC_BOOT
//						 - add thisNN routine
//						 - add code for OPC_RQNPN
//						 - add doError for command error response
// 02/01/12 Pete Brownlow - Process WCVOA
// 04/11/23 Pete Brownlow - Process ALOC to put loco into shuttle

#include "project.h"


#pragma udata MAIN_VARS

void cmd_cv(void);

#pragma code APP

//
// parse_cmd()
//
// Decode the OPC and call the function to handle it.
//
void parse_cmd(void) {

    WORD addr;
    BYTE session, speed, shuttleIndex;
    enum glocModes  requestmode;

    mode_word.s_full = 0;

    switch(rx_ptr->d0) {			
        case OPC_DSPD:
			// Loco speed/dir
			session = rx_ptr->d1;
			speed = rx_ptr->d2;
			speed_update(session, speed);
                        break;

        case OPC_DFUN:
            // Loco sped/dir or functions
            function_update(rx_ptr->d1, rx_ptr->d2, rx_ptr->d3);
            break;

	case OPC_DFNON:				
	case OPC_DFNOF:				
            loco_function((rx_ptr->d0 == OPC_DFNON ? funcon : funcoff ), rx_ptr->d1, rx_ptr->d2 );
            break;

        case OPC_DKEEP:
            // Session keep alive message
            session = rx_ptr->d1;
            keep_alive(session);
            break;

        case OPC_QCVS:
        case OPC_WCVS:
        case OPC_WCVO:
        case OPC_WCVB:
        case OPC_WCVOA:
            // CV programming
            cmd_cv();
            break;

        case OPC_RLOC:
        case OPC_GLOC:
            // Request session for loco
            addr = (unsigned int) rx_ptr->d1<<8;
            addr |= rx_ptr->d2;

            if (rx_ptr->d0 == OPC_GLOC)
            {
                requestmode = (rx_ptr->d3);
            }
            else
                requestmode = glocNormal;
            
            queue_add(addr, requestmode, (ModNVPtr) cmdNVptr);

            break;

        case OPC_STMOD:
            throttle_mode();
            break;
//?????  must also implement OPC_DFLG
            
        case OPC_KLOC:
            // Release engine by session number
            session = rx_ptr->d1;
            release_loco(session, cmdNVptr->userflags.shuttles);
            break;

        case OPC_QLOC:
            // Query engine by session number
            session = rx_ptr->d1;
            query_session(session);
            break;

        case OPC_ALOC:
            // Allocate loco to shuttle
            session = rx_ptr->d1;
            shuttleIndex = rx_ptr->d2;
            if (cmdNVptr->userflags.shuttles)
            {    
                populate_shuttle(session, shuttleIndex, FALSE);
                setShuttleNVs( shuttleIndex );
            }    
                    
            break;

        case OPC_RDCC3:
        case OPC_RDCC4:
        case OPC_RDCC5:
        case OPC_RDCC6:
            // Send DCC packet
            dcc_packet();
            break;

        case OPC_RTOF:
        case OPC_RTON:
            // Track power control
            power_control(rx_ptr->d0 == OPC_RTON);
            break;

        case OPC_RESTP:
            // Emergency stop all
            
            stopAll();
            break;

      // case OPC_QCON:   - no longer in the spec
            // We only support advanced consisting so query consist
            // is not supported
            // Can't give an address so send 0
            // Tx1[d0] = OPC_ERR;
            // Tx1[d1] = 0;
            // Tx1[d2] = 0;
            // Tx1[d3] = ERR_NO_MORE_ENGINES;
            // can_tx(4);
            // break;

        case OPC_PCON:
            // Add to consist
            consist_add();
            break;

// Don't support this since cab will have a handle for stack entry with
// consist address in it. You cannot write to CV19 via consist address.
// Instead, cab will select real loco address and send new consist address=0
//        case OPC_KCON:
//            // Remove from consist
//            consist_remove();
//            break;

        case OPC_RSTAT:
            // Send command station status
            send_stat();
            break;

        case OPC_BOOT:
            // Enter bootloader mode if NN matches
            if (thisNN() == 1)
	        {
                ee_write((unsigned char)(&bootflag), 0xFF);
                Reset();
            }
            break;
            
        default: if ((rx_ptr->d0 || 0xE0) == 0xFF) 
                    {
			parse_extended_cmd();
                    }
                    else if (!parse_FLiM_cmd())
                    {
                        parse_cbus_event();
                    }
                    break;
                  
    }

    if (NV_changed) {
	init_isr_high( cmdNVptr );	// Reinitialise as something may have been changed
    }

    if (mode_word.s_full == 0) {
        // No problem so retire RX buffer
        Nop();
        rx_ptr->con = 0;
        if (op_flags.bus_off) {
            // At least one buffer is now free
            op_flags.bus_off = 0;
            PIE3bits.FIFOWMIE = 1;
            setCbusOn();
    }
    } else {
        // Couldn't allocate in S queue so keep buffer valid and try
        // again next time round
        ;
    }
}


//
// parse_extended_cmd()
//
// Decode the OPC and call the function to handle it.
//
void parse_extended_cmd(void) {

    unsigned int addr;
	BOOL force;
 
    switch(rx_ptr->d0) {		
//        case OPC_EXTC5:
//			// Extended command with 5 data bytes
//			switch(rx_ptr->d1) {
//				case SUBOPC_XRLOC:
//					// Extended request loco
//					addr = (unsigned int)((rx_ptr->d4<<8) | rx_ptr->d5);
//					force = rx_ptr->d6 && 0x01;
//					queue_add(addr, force );
//					break;
//
//				default: break;
//			} // switch subopcode


  
        default: break;              
	} // switch main opcode

} // parse_extended_cmd


//
// parse_cbus_event()
//
// Check for a CBUS event and call the function to handle it.
//
BOOL parse_cbus_event(void) {
    
    BOOL cmd_processed;

    cmd_processed = TRUE;
	
    switch(rx_ptr->d0) {
        case OPC_ACON:
		case OPC_ACOF:
        case OPC_ASON:
        case OPC_ASOF:
        case OPC_ARON:
		case OPC_AROF:
        case OPC_ARSON:
        case OPC_ARSOF:            
	//	case OPC_ACON1:
	//	case OPC_ACOF1:
	//	case OPC_ACON2:
	//	case OPC_ACOF2:
	//	case OPC_ACON3:
	//	case OPC_ACOF3:
            // CBUS event
          	cbus_event(rx_ptr, (ModNVPtr) cmdNVptr);
			break; 

        default: cmd_processed = FALSE;
                break;
    }

	return( cmd_processed );
} // parse_cbus_event








