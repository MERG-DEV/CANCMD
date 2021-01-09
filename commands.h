#ifndef __COMMANDS_H
#define __COMMANDS_H

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

// 06/04/11 Roger Healey - Add fn prototypes for doRqnpn, doError and thisNN

// mode values
#define DIRECT_BYTE 0
#define DIRECT_BIT 1
#define PAGED 2
#define REGISTER 3
#define ADDRESS 4
#define PWR_OFF 10
#define PWR_ON 11

void parse_cmd(void);
void parse_extended_cmd(void);


BOOL parse_cbus_event(void);


// Hard coded events for Command station management

#define HC_CS_NODE      452
#define HC_STOP_ALL     103

// Temp hard coded stuff for shuttle proof of concept testing

#if DRShuttle

#define SH_POC_EN_NODE  115             // Node 
#define	SH_POC_ENABLE_EN 6              //   and event to enable shuttle

#define SH_FWD_NODE     115             // Node
#define SH_FWD_EN       1               //     and event base for forward end reversing sensor (DTC on Dablow Jn)

#define SH_REV_NODE     116             // Node
#define SH_REV_EN       6               //     and event base for reverse end reversing sensor 9Hectors on Dablow jn)

#define SH_BUT_NODE     120             // Node for push buttons
#define SH_BUT_EN       6               // Event base for shuttle buttons
#define SH_HONK_EN      9               // 4th button not currently fitted - defined for future use 

#define POC_MAX         2

#else  // Test Board and any other test shuttle

#define SH_POC_EN_NODE  162             // Node 
#define	SH_POC_ENABLE_EN 5              //   and event to enable shuttle

#define SH_FWD_NODE     162             // Node
#define SH_FWD_EN       2               //   and event for forward end reversing sensor

#define SH_REV_NODE     161             // Node
#define SH_REV_EN       2               //   and event for reverse end reversing sensor

#define SH_BUT_NODE     120             // Node for push buttons
#define SH_BUT_EN       6               // Event base for shuttle buttons
#define SH_HONK_EN      5               // Event for honk button

#define POC_MAX         2               // Maximum shuttle no.

#define TOTI_DEBOUNCE   10              // Counts round main loop for train detector input debounce

#endif


#endif
