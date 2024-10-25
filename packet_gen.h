#ifndef __PACKET_GEN_H

//
// MERG CBUS DCC Command Station/Programmer - declarations for DCC packet generation

//      All source code is copyright (C) the author(s) concerned
//	Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
//      Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
//      Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2012 Pete Brownlow  merg@upsys.co.uk
//
/*
 *   This work is licensed under the:
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
*/

//
// The Refresh queue ( or Q queue) holds entries for loco
// control data that is continually sent to the track.
//
// 8 byte entries:
//				; byte 1: status
//				; byte 2: addr high
//              ; byte 3: addr low
//              ; byte 4: speed/dir in dcc 128 step format
//				; byte 5: Fn group 1 (F0 to F4)
//				; byte 6: Fn group 2 (F5 to F8)
//				; byte 7: Fn group 2a (F9 to F12)
//				; byte 8: slot timeout (if used)
//

// refresh queue status byte
typedef union {
	struct {
		unsigned share_count:2;     // Count of throttles sharing
		unsigned cached:1;          // Slot released but info cached for next use of this loco
		unsigned shuttle:1;         // In use for a shuttle
		unsigned dispatched:1;      // Dispatched (not attached to a cab)
		unsigned throttle_mode:2;   // Number of speed steps - see STMOD in cbusdefs
		unsigned valid:1;
	} ;
	BYTE byte;
} q_slot_status;


// Function byte 1

typedef union {
	struct {
		unsigned	fngroup:3;
		unsigned	FLvalue:1;
		unsigned	F4value:1;
		unsigned	F3value:1;
		unsigned	F2value:1;
		unsigned	F1value:1;
	} ;

	BYTE byte;
} fn1status;

// Function byte 2

typedef union {
	struct {
		unsigned	fngroup:3;
		unsigned	fnrange:1;
		unsigned	F8value:1;
		unsigned	F7value:1;
		unsigned	F6value:1;
		unsigned	F5value:1;
	} ;

	BYTE byte;
} fn2status;

// Function byte 2a

typedef union {
	struct {
		unsigned	fngroup:3;
		unsigned	fnrange:1;
		unsigned	F12value:1;
		unsigned	F11value:1;
		unsigned	F10value:1;
		unsigned	F9value:1;
	} ;

	BYTE byte;
} fn2astatus;



// A single queue entry
typedef struct refresh_queue {
    q_slot_status status;
    dcc_address address;
    unsigned char speed;
    fn1status fn1;
    fn2status fn2;
    fn2astatus fn2a;
    unsigned char timeout;
} refresh_queue_t;

// The array of 32 entries for the loco queue

extern refresh_queue_t q_queue[MAX_HANDLES];
extern unsigned char q_idx;
extern unsigned char q_state;

// The send queue (or S queue) holds DCC formatted packets to
// be sent "immediately" to the track.
//
// Immediate packets include speed and funtion updates and service
// mode packets. In future they could include accessory control
// packets.
//
// 8 byte entries:
//				; byte 1: bit 8: status
//				; byte 1: bits 2:0: byte count
//				; byte 2:7: data
//				; byte 8: bits 2:0: repeat count
//

// send queue status byte
typedef union {
	struct {
		unsigned byte_count:3;
		unsigned shuttle:1;		   // In use for a shuttle
		unsigned dispatched:1;	   // Dispatched (not attached to a cab)
		unsigned throttle_mode:2;  // Number of speed steps - see STMOD in cbusdefs
		unsigned valid:1;
	} ;
	unsigned char byte;
} s_slot_status;



// A single queue entry
typedef struct dcc_queue {
	s_slot_status status;
    unsigned char d[6];
    unsigned char repeat;
} dcc_queue_t;


enum funcops
{
	funcoff = 0,
	funcon = 1,
	functog = 2
};

enum glocModes
{
    glocNormal = 0,
    glocSteal = 1,
    glocShare = 2
};

// The array of 16 entries for the temporary queue
//extern struct dcc_queue s_queue[16];
extern dcc_queue_t s_queue[16];
extern unsigned char s_head;
extern unsigned char s_tail;
extern unsigned char s_cnt;

void packet_gen( ModNVPtr NVPtr);
BYTE queue_add(WORD addr, enum glocModes requestmode, ModNVPtr cmdNVPtr );
void send_ploc( BYTE handle );
void force_release( BYTE handle, BOOL stolen );
void throttle_mode(void);
void loco_function(enum funcops dofunc, BYTE session, BYTE funcnum);
BYTE set_funcop( BYTE *funcbyte, enum funcops dofunc, BYTE funcvalues );
void speed_update( BYTE session, BYTE speed );
void function_update( BYTE session, BYTE funcrange, BYTE funcvalues );
void queue_update(BYTE session, BYTE funcrange, BYTE funcvalues );
void query_session(BYTE session);
void release_loco(BYTE session, BOOL shuttlesActive );
void cache_session( BYTE session );
void purge_session(BYTE session);
void check_session_timeouts( ModNVPtr NVPtr );
void keep_alive(BYTE session);
void dcc_packet(void);
void em_stop(void);
void broadcast_stop(void);
void consist_add(void);
void consist_remove(void);
void ops_write(dcc_address ops_address, WORD cv_num, BYTE cv_data, BYTE write_mode );
void dccAccessoryWrite(WORD acc_num, BOOL accOn);
BOOL dccAccessoryRoute(WORD event_num, BOOL accOn);


#define __PACKET_GEN_H
#endif
