#ifndef __CAN_H
#define __CAN_H

/*

 Copyright (C) Pete Brownlow 2012   software@upsys.co.uk

  CBUS Command Station - definitions for common CAN routines

 This code is for a CBUS DCC Command Station

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
#include "devincs.h"
#include <GenericTypeDefs.h>


enum bufbytes {
	con=0,
	sidh,
	sidl,
	eidh,
	eidl,
	dlc,
	d0,
	d1,
	d2,
	d3,
	d4,
	d5,
	d6,
	d7
};


// Data structures for transmit and receive buffers

typedef union
{
    struct
    {
        unsigned    dlcBits:4;
        unsigned    reservedBits:2;
        BOOL        rtrBit:1;
    };
    BYTE    dlc;
} DLCByte;


typedef struct {
	unsigned char con;
	unsigned char sidh;
	unsigned char sidl;
	unsigned char eidh;
	unsigned char eidl;
	DLCByte       dlc;
	unsigned char d0;
	unsigned char d1;
	unsigned char d2;
	unsigned char d3;
	unsigned char d4;
	unsigned char d5;
	unsigned char d6;
	unsigned char d7;
} ecan_rx_buffer;

extern ecan_rx_buffer* rx_ptr;
extern near BYTE Tx1[14];

extern BYTE Latcount;
extern BYTE BeepCount;
extern BYTE canTransmitTimeout;


void initCan(BYTE EEcanID);
void initCanTxBuffers(BYTE EEcanID);
void clearCanRXBuffers(void);
BOOL isCanMsgReceived(void);
void checkIncomingCanID(void);
void processRTR(void);
void sendTX1(void);
void serviceCanInterrupts(void);
void clearFifo(void);
BYTE* _PointBuffer(BYTE b);


#endif	// __CAN_H

