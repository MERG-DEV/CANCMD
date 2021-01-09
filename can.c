/*
        Code based on Mike Bolton's canacc8 for which due credit is given. 
        Modifications and conversion to C18 Copyright (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 Microchip C18 source for common CAN routines

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



#include "can.h"
#include "romops.h"
// #include "project.h"

// Memory buffers for use with CAN

#pragma udata access VARS

near BYTE Tx1[14];          // Transmit buffer

#pragma udata MAIN_VARS

ecan_rx_buffer * rx_ptr;    // Receive buffer
BYTE            Latcount;
BYTE            canTransmitTimeout;
BOOL            canTransmitFailed;
BYTE            BeepCount;
BOOL            enumerationRequired;
BOOL            enumerationActive;


#pragma code APP

/*
 * Common CAN setup
 */
void initCan(BYTE EEcanID) {
    enumerationRequired = FALSE;
    enumerationActive = FALSE;

    canTransmitTimeout = 0;

    CANCON = 0b10000000;        // CAN to config mode
    while (CANSTATbits.OPMODE2 == 0)
        // Wait for config mode
        ;
    

        // ECAN mode 2 with FIFO
        ECANCON = 0b10110000;       // FIFOWM = 1 (one entry)
                                    // EWIN default
        BSEL0 = 0;                  // All buffers to receive so
                                    // FIFO is 8 deep
        RXB0CON = 0b00000000;       // receive valid messages
 

	/*
	 * Baud rate setting.
         * Sync segment is fixed at 1 Tq
	 * Total bit time is Sync + prop + phase 1 + phase 2
	 * or 16 * Tq in our case
	 * So, for 125kbits/s, bit time = 8us, we need Tq = 500ns
	 * Fosc is 32MHz (8MHz + PLL), Tosc is 31.25ns, so we need 1:16 prescaler
	 */
	// prescaler Tq = 16/Fosc
   //  BRGCON1 = 0b00000011;  old value
    BRGCON1 = 0b00000111;
	// freely programmable, sample once, phase 1 = 4xTq, prop time = 7xTq
    BRGCON2 = 0b10011110;
	// Wake-up enabled, wake-up filter not used, phase 2 = 4xTq
    BRGCON3 = 0b00000011;
	// TX drives Vdd when recessive, CAN capture to CCP1 disabled
    CIOCON = 0b00100000;
    
    RXFCON0 = 1;                // Only filter 0 enabled
    RXFCON1 = 0;
    SDFLC = 0;
    
    // Setup masks so all filter bits are ignored apart from EXIDEN
    RXM0SIDH = 0;
    RXM0SIDL = 0x08;
    RXM0EIDH = 0;
    RXM0EIDL = 0;
    RXM1SIDH = 0;
    RXM1SIDL = 0x08;
    RXM1EIDH = 0;
    RXM1EIDL = 0;
    
    // Set filter 0 for standard ID only to reject bootloader messages
    RXF0SIDL = 0x80;

    // Link all filters to RXB0 - maybe only neccessary to link 1
    RXFBCON0 = 0;
    RXFBCON1 = 0;
    RXFBCON2 = 0;
    RXFBCON3 = 0;
    RXFBCON4 = 0;
    RXFBCON5 = 0;
    RXFBCON6 = 0;
    RXFBCON7 = 0;

    // Link all filters to mask 0
    MSEL0 = 0;
    MSEL1 = 0;
    MSEL2 = 0;
    MSEL3 = 0;

    BIE0 = 0;       // No Rx buffer interrupts - assume FIFOWM will interrupt
    TXBIE = 0;      // No Tx buffer interrupts

    // Clear RXFUL bits
    RXB0CON = 0;
    RXB1CON = 0;
    B0CON = 0;
    B1CON = 0;
    B2CON = 0;
    B3CON = 0;
    B4CON = 0;
    B5CON = 0;

    CANCON = 0;                	// Out of CAN setup mode


    // Setup transmit buffers witrh can_id etc
    initCanTxBuffers(EEcanID);
     // clear the fifo receive buffers
    clearCanRXBuffers();
}

// Clear CAN receive FIFO

void clearCanRXBuffers(void)

{
   while (isCanMsgReceived()) {
        rx_ptr->con = 0;
   }
}


// Initilase the CAN transmit buffers, one for general use,
// one dedicated to transmit after TX error or overflow and one
// dedicated to responding to RTR
// These buffers are intialised at startup and whenever the can id is changed

void initCanTxBuffers(BYTE EEcanID)
{
    BYTE can_ID;

    // ??? tidy this up to work out basic sidl/sidh values once into local var
    //???  also declare those priroiry masks as costants

    can_ID = ee_read( EEcanID );

    Tx1[con] = 0;
    Tx1[sidh] = 0b10110000 | (can_ID & 0x78) >>3;
    Tx1[sidl] = (can_ID & 0x07) << 5;

    // Setup TXB0 with high priority OPC_HLT
//    TXB0SIDH = 0b01110000 | (can_ID & 0x78) >>3;
//    TXB0SIDL = (can_ID & 0x07) << 5;
//    TXB0DLC = 1;
//    TXB0D0 = OPC_HLT;

    // Setup TXB2 with response to RTR frame
    TXB2SIDH = 0b10110000 | (can_ID & 0x78) >>3;
    TXB0SIDL = (can_ID & 0x07) << 5;
    TXB0DLC = 0;   // no data, RTR bit cleared
}


//
// isCanMsgReceived()
//
// Check if ECAN FIFO has a valid receive buffer available and
// preload the pointer to it. Also handles response to incoming RTR frame
// ??? Check we are ignoring extended frames

BOOL isCanMsgReceived(void) {

  BOOL isBufferReady = FALSE;
  BOOL isMsgInBuffer;

  while (!isBufferReady)
  {

    switch (CANCON & 0b00000111) {
        case 0:
            rx_ptr = (ecan_rx_buffer *)&RXB0CON;
            break;

        case 1:
            rx_ptr = (ecan_rx_buffer *)&RXB1CON;
            break;

        default:
            // Remaining buffers are 16 bytes long starting at
            // pointer value 2
            rx_ptr = (ecan_rx_buffer *)(&B0CON + (((CANCON - 2) & 0b00000111)<<4));
            break;

    }


    if (isMsgInBuffer = (rx_ptr->con & 0x80) )
    {
        // current buffer is not empty
        checkIncomingCanID();

        if (rx_ptr->dlc.rtrBit)
            processRTR();
        else if ((rx_ptr->dlc.dlcBits == 0) || (rx_ptr->sidl & 0x08) ) // Check for zero length payload or extended frame
            rx_ptr->con = 0;    // Just retire buffer for now
        else
            isBufferReady = TRUE;

    }
    else
        isBufferReady = TRUE;
  } // loop until buffer ready to be returned
        
  return (isMsgInBuffer != 0);
 
}


/*  Self enumeration   */

void checkIncomingCanID(void)

{

}

void processRTR(void)
{
    rx_ptr->con = 0;    // Just retire buffer for now
}

void self_enumerate()

{

}


/*
 * Send contents of Tx1 memory buffer via CAN TXB1 transmit buffer
 */
void sendTX1(void) {
	unsigned char tx_idx;
	unsigned char * ptr_fsr1;
	unsigned char i;

	Latcount = 10;
	canTransmitTimeout = 2;	// half second intervals
	canTransmitFailed = FALSE;

        while ((TXB1CONbits.TXREQ) && (!canTransmitFailed) && (canTransmitTimeout != 0)) {
            INTCONbits.GIEH = 1;		// Make sure interrupts enabled
            INTCONbits.GIEL = 1;		//
            Nop();
	}

        Tx1[sidh] &= 0b00001111;		// clear old priority
	Tx1[sidh] |= 0b10110000;		// low priority

    // Load TXB1
	tx_idx = 0;
	ptr_fsr1 = &TXB1CON;

	for (i=0; i<14; i++) {
		*(ptr_fsr1++) = Tx1[tx_idx++];
	}

	canTransmitTimeout = 2;	// half second intervals
	canTransmitFailed = FALSE;

        // ??? Should not really need to do this again having already check it before loading buffer???

	while((TXB1CONbits.TXREQ) && (!canTransmitFailed) && (canTransmitTimeout != 0)) {
            INTCONbits.GIEH = 1;		// Make sure interrupts enabled
            INTCONbits.GIEL = 1;		//
            Nop();
	}

	TXB1CONbits.TXREQ = 1;
}


// Called from ISR to service CAN interrupts

void serviceCanInterrupts(void)
{
    // If FIFO watermark interrupt is signalled then we copy the contents
    // of the harddware fifo into the software fifo array. If the software fifo is full,
    // nothing more we can do except let hardware fifo continue to fill up.

    
    if (PIR3bits.FIFOWMIF == 1) {
        TXB0CONbits.TXREQ = 1;
      //  op_flags.bus_off = 1;
        PIE3bits.FIFOWMIE = 0;
    }
    if (PIR3bits.ERRIF == 1) {
		if (TXB1CONbits.TXLARB) {  // lost arbitration
			if (Latcount == 0) {	// already tried higher priority
				canTransmitFailed = TRUE;
				if (BeepCount == 0) {
					BeepCount = 2;		// 2 beeps for CAN transmit failure

				}
				TXB1CONbits.TXREQ = 0;
			}
			else if ( --Latcount == 0) {	// Allow tries at lower level priority first
				TXB1CONbits.TXREQ = 0;
				Tx1[sidh] &= 0b00111111;		// change priority
				TXB1CONbits.TXREQ = 1;			// try again
			}
		}
		if (TXB1CONbits.TXERR) {	// bus error
			canTransmitFailed = TRUE;
			if (BeepCount == 0) {
				BeepCount = 2;		// 2 beeps for CAN transmit failure
			}
			TXB1CONbits.TXREQ = 0;
		}

    }
}