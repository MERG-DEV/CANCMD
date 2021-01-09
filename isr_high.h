#ifndef __ISR_H

/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 CBUS Command Station - definitions for high priority Interrupt Service Routine

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

#define LONG_PRE	22	// Increased from 20 - PNB 16/4/11 to meet NMRA spec 
#define NORM_PRE	18  // Increased from 14 - PNB 14/3/16 to allow time for railcom cutout - increased further 16->18  28/7/20 to ensure compliance with current railcom spec. Check DCC_CUTOUT_END value if you change this.

#define dcc_pre_m   NORM_PRE
#define dcc_pre_s	LONG_PRE

// Preamble count is decremented each full cycle, 116us each time. 
// Railcom Cutout should end between 454 and 488uS after the stop bit edge that triggered the cutout
// 4 counts at 116uS per count gives 464uS, which is within the permitted range, so with 18 preambles, the preamble count should be at 14 when the cutout ends.
        
#define DCC_CUTOUT_END  NORM_PRE - 4

typedef	union
{
    struct
    {
        BOOL    railcomEnabled:1;
        BOOL 	waitHalfPeriod:1;
        BOOL	atHalfPeriod:1;
        BOOL	spare:5;
    } ;
    BYTE	byte;
} RailcomFlags;


// ISR prototype 
extern void isr_high(void);
extern void isr_low(void);

extern near BOOL swap_op;

extern near WORD svc_ovld_delay;
extern near WORD sumsvc;
extern near WORD avesvc;
extern near WORD avemain;

extern near BYTE iccq;
extern near BYTE bit_flag_s;
extern near BYTE dcc_bytes_s;
extern near BYTE bit_flag_m;
extern near BYTE dcc_bytes_m;


extern unsigned short long slot_timer;


void init_isr_high( ModNVPtr NVPtr );
BOOL set_output( ModNVPtr NVPtr );
void setbeep( ModNVPtr NVPtr );


#define __ISR_H
#endif
