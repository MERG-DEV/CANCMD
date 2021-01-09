#ifndef __PROGRAM_CMDS_H

/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 CBUS Command Station -  definitions for programming commands

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


extern unsigned char sm_handle;
extern unsigned int cv;
extern unsigned char cv_register;
extern unsigned char cv_page;
extern unsigned char mode;
extern unsigned char cv_data;
extern unsigned char prog_state;


#define CV_IDLE             0
#define CV_PON1             1
#define CV_CHECK_OVLD       2
#define CV_WRITE_BIT_REC    3
#define CV_WRITE_BIT        4
#define CV_READ_BIT_V_1     5
#define CV_READ_BIT_1_REC   6
#define CV_READ_BIT_V_0     7
#define CV_READ_BIT_0_REC   8
#define CV_READ_BYTE_START  9
#define CV_READ_PAGE_SENT   10
#define CV_PAGE_SENT        11
#define CV_PAGE_REC         12
#define CV_READ             13
#define CV_REC              14
#define CV_POFF             15


void cv_sm(ModNVPtr NVPTR);
void cmd_cv(void);

#define __PROGRAM_CMDS_H
#endif
