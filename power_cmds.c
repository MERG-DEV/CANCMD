/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 MERG CBUS DCC Command Station/Programmer - track power control

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

#pragma code APP

//
// power_control()
//
// Turn track on or off.
//
void power_control(BOOL powerOn) {
    if (powerOn) {
        // Turn on main track power
        op_flags.op_pwr_m = 1;

        // Acknowledge it
        Tx1[d0] = OPC_TON;
    } else  {
        // Turn off main track power
        op_flags.op_pwr_m = 0;

        // Acknowledge it
        Tx1[d0] = OPC_TOF;
    }
    sendCbusMsg();
}

void stopAll( void )
{
    em_stop();              // Set all sessions to emergency stop and send DCC stop broadcast
    sendCbusOpc(OPC_ESTOP); // Send track stopped CBU in response
    if ( cmdNVptr->opflags.trkoffwithstopall)
        power_control( FALSE );
}


//
// Power On Cycle
//
// RP923 calls for at least 20 valid packets with a check for overload of 250mA
// sustained after 100ms. 14 reset packets gives us approx 105ms, after which the
// decoder quiescent current is logged and checked for overload. We then enable
// gross overload checking in the A/D routines and a further 6 reset packets
// complete the power on cycle.
//
// [1.5] Add another 15 reset packets for QSI V6 steam decoders
//
//void power_on_cycle() {
//  iccq = 0;
//  // [rev r] initialize current measurement
//  sum = 0;
//  ave = 0;
//  op_flags.op_pwr_s = 1;
//  dcc_flags.dcc_overload = 0;
//  dcc_flags.dcc_check_ovld = 0;
//  dcc_flags.dcc_check_ack = 0;
//  dcc_flags.dcc_ack = 0;
//  packet_reset(14);
//  iccq = ave;
//  if (iccq > I_OVERLOAD) {
//    op_flags.op_pwr_s = 0;
//    dcc_flags.dcc_overload = 1;
//    return;
//  }
//  dcc_flags.dcc_check_ovld = 1;
//  dcc_flags.dcc_check_ack = 1;
////  packet_reset(6);
//  packet_reset(21);
//  // [rev r] wait until all reset packets complete before final sampling of ICCQ
//  while (dcc_flags.dcc_rdy_s == 0)
//    ;
//  iccq = ave;
//}

//
// power_off_cycle()
//
//void power_off_cycle(void) {
//  recovery_time();
//  op_flags.op_pwr_s = 0;
//}

//
// packet_reset(unsigned char i)
//
// Place i reset packets in the DCC transmit buffer
//
void packet_reset(unsigned char i) {
    while (dcc_flags.dcc_rdy_s == 0) {
    	INTCONbits.GIEH = 1;		// Make sure interrupts enabled
    	INTCONbits.GIEL = 1;		// ??? Probably should have some sort of timeout on these potentially infinite loops
		Nop();
	}
      
    dcc_buff_s[0] = dcc_buff_s[1] = dcc_buff_s[2] = 0;
    dcc_buff_s[6] = i;            // repeat
    dcc_bytes_s = 3;
//    dcc_pre_s = LONG_PRE;
    dcc_flags.dcc_rdy_s = 0;
}

//
// Place an idle packet in the DCC transmit buffer
//
void packet_idle(unsigned char i) {
      while (dcc_flags.dcc_rdy_s == 0) {
    	INTCONbits.GIEH = 1;		// Make sure interrupts enabled
    	INTCONbits.GIEL = 1;		// ??? Probably should have some sort of timeout on these potentially infinite loops
		Nop();
	  }
        
      dcc_buff_s[0] = dcc_buff_s[2] = 0xff;
      dcc_buff_s[1] = 0;
      dcc_buff_s[6] = i;        // repeat
      dcc_bytes_s = 3;
//      dcc_pre_s = LONG_PRE;
      dcc_flags.dcc_rdy_s = 0;
}

