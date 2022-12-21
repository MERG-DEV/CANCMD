#ifndef __CANCMD_H
#define __CANCMD_H

/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 CBUS Command Station - project definitions 

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

// define SWAP_OUTPUTS to put main track output through on board
// booster, purely for testing, not suitable for normal use since
// programming track, through external booster, will have no current
// sensing feedback
//#define SWAP_OUTPUTS

//
// Current sensing for SPROG
// 5V reference => 5/1024 = 4.88mV resolution
// Sense resistor is 0R47
// so 60mA is 28.2mV Vsense => 5 steps
// 250mA overload is 117.5mV Vsense => 24 steps

// 06/04/11 Roger Healey - Add external reference to params
//
// Most of these definitions are now in Node Variables

// #define I_ACK_DIFF 5	// No. steps for additional 60ma ACK pulse
#define I_OVERLOAD 24
// #define I_DEFAULT 96
// #define I_LIMIT 768

// CANCMD specific definitions

#define MAX_HANDLES 32
#define MAX_SHUTTLES 15

//
// Flags register used for DCC packet transmission
//
typedef union {
    struct {
        unsigned dcc_rdy_s:1;		    // set if Tx ready for a new packet
        unsigned dcc_long_pre:1;        // set forces long preamble
        unsigned dcc_retry:1;
        unsigned dcc_ack:1;
        unsigned dcc_overload:1;        // set if overload detected
        unsigned dcc_check_ack:1;
        unsigned dcc_check_ovld:1;
        unsigned :1;
        unsigned dcc_rdy_m:1;
        unsigned dcc_reading:1;
        unsigned dcc_writing:1;
        unsigned dcc_cv_no_ack:1;
        unsigned dcc_rec_time:1;
        unsigned railcom_cutout_active:1;
        unsigned dcc_em_stop:1;
        unsigned dcc_test_outputs:1;
    } ;
    unsigned int word;
} DccFlags;

//
// MODE_WORD flags
//
typedef union {
    struct {
        unsigned boot_en:1;
        unsigned :1;
        unsigned s_full:1;
        unsigned :1;
        unsigned :1;
        unsigned ztc_mode:1;	// ZTC compatibility mode
        unsigned direct_byte:1;
        unsigned :1;
    } ;
    unsigned char byte;
} ModeWord;

//
// OP_FLAGS for DCC output
//
typedef union {
    struct {
        unsigned op_pwr_s:1;
        unsigned op_bit_s:1;
        unsigned op_pwr_m:1;
        unsigned op_bit_m:1;
        unsigned bus_off:1;
        unsigned slot_timer:1;
        unsigned can_transmit_failed:1;
        unsigned beeping:1;
    } ;
    unsigned char byte;
} OpFlags;


// DCC address high byte
typedef union {
    // As six address bits plus two bits that should be high for
    // a long address
    struct {
        unsigned hi:6;
        unsigned long0:1;
        unsigned long1:1;
    } ;
    // as a single byte
    unsigned char byte;
} dcc_address_high;

// DCC address
typedef union {
    // the two bytes
    struct {
        unsigned char addr_lo;
        dcc_address_high addr_hi;
    } ;
    // the whole thing as an int
    unsigned int addr_int;
} dcc_address;

typedef union
{
    struct
    {
        BYTE speed:7;
        BOOL direction:1;
    };
    BYTE    velocity;
} DCCSpeed;


extern near DccFlags    dcc_flags;
extern near OpFlags     op_flags;

extern near BYTE        dcc_buff_s[7];
extern near BYTE        dcc_buff_m[7];

extern ModeWord         mode_word;
extern BYTE             tmr0_reload;
extern BYTE             startupTimer;

#endif	// __CANCMD_H
