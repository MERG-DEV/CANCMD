/*
 
        Original code up to version 2 (C) 2009 SPROG DCC  http://www.sprog-dcc.co.uk   sprog@sprog-dcc.co.uk
        Changes for version 3 (C) 2011 Pete Brownlow      merg@uspys.co.uk
        Support for FLiM, shuttles, DCC accessories and other changes for ver 4 (C) 2011-2017 Pete Brownlow  merg@upsys.co.uk

 MERG CBUS DCC Command Station/Programmer - DCC packet generation

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
//#include "commands.h"
//#include "packet_gen.h"


//
// 27/6/11  Pete Brownlow - Heartbeat is now half second, counts adjusted so slot timeout unchanged
// 02/01/12 Pete Brownlow - ops-write modified to take parameters, to support WCVOA

#pragma udata Q_QUEUE
// Linker script puts this in GPR1
// 32 entry Q queue for regularly refreshed loco packets
refresh_queue_t q_queue[MAX_HANDLES];

#pragma udata S_QUEUE
// Linker script puts this in GPR2
// 16 entry S queue for one off packets
dcc_queue_t s_queue[16];

#pragma udata MAIN_VARS
// indices for the queues
unsigned char q_idx;
unsigned char q_state;
unsigned char s_head;
unsigned char s_tail;
unsigned char idle_next;

#pragma code APP

void send_s(CancmdDbugFlags debugflags);
void send_q(CancmdDbugFlags debugflags, BOOL railcomEnabled);
void send_idle(void);


//
// Scale 128 step format speed to 28 step format
//	??? step through this and check all permutations

unsigned char step28(unsigned char spd) {
    unsigned char ret = 0b01000000; // reverse
    if (spd > 0x7f) {
        ret = ret | 0b00100000; // forwards
    }
    if ((spd &= 0x7f) == 1) { // Remove speed bit - then is it emerg stop?
        return ret | 0x01; // Em stop
    }

    if ((spd & 0b01111100) == 0b00000100) // If speed will scale to 1, then send speed 2
    {
        spd++;
    }

    ret = ret | (spd >> 3); // shift 4 MSBs
    if (spd & 0b00000100) {
        ret = ret | 0x10; // Put LSB in bit 4
    }
    return ret;
}

//
// Scale 128 step format speed to 14 step format
//

unsigned char step14(unsigned char spd) {
    unsigned char ret = 0b01000000; // reverse
    if (spd & 128) {
        ret = ret | 0b00100000; // forwards
    }
    if ((spd &= 0x7f) == 1) {
        return ret | 0x01; // Em stop
    }

    if ((spd & 0b01111000) == 0b00001000) // If speed will scale to 1, then send speed 2
    {
        spd++;
    }


    return ret | (spd >> 3); // shift 4 MSBs
}

//
// em_stop()
//

void em_stop(void) {
    unsigned char i;
    // Set bit for broadcast for next packet
    dcc_flags.dcc_em_stop = 1;
    // Set all sessions to Emergency stop, preserving direction bit.
    for (i = 0; i < MAX_HANDLES; i++) {
        q_queue[i].speed = (q_queue[i].speed & 0x80) | 1;
    }
}

//
// broadcast_stop(void)
//
// Send 6 broadcast emergency stop packets
//

void broadcast_stop(void) {
    dcc_buff_m[0] = 0;
    dcc_buff_m[1] = 0b01110001;
    dcc_buff_m[2] = 0b01110001;
    dcc_buff_m[6] = 6; // repeat 6 times
    dcc_bytes_m = 3;
    // hand off buffer	
    dcc_flags.dcc_rdy_m = 0;
}

//
// packet_gen()
//
// Called from the input wait loop to handle main track packet
// generation.
//
// 14/28 speed step is a 3 or 4 byte packet <address> <01DUSSSS> <error>
// ought to be able to set U to FL control
// 128 speed step is a 4 or 5 byte packet <Address> <00111111> <speed>
// <error>
//
// <Address> can be 7 bit in a single byte <0aaaaaaa> or 14 bit in two 
// bytes <11aaaaaa> <aaaaaaaa>. The first byte of a 14 bit address has
// a valid range of 0xc0 to 0xe7.
//
// Packets in the S queue have priority to ensure we respond to throttle
// speed changes as soon as possible.
//
// Loco speed/dir is refreshed every time round.
//

void packet_gen(ModNVPtr NVPtr) {
    if (dcc_flags.dcc_em_stop == 1) {
        // Broadcast emergency stop
        broadcast_stop();
        dcc_flags.dcc_em_stop = 0;
    } else if (idle_next == 0) {
        if (s_queue[s_tail].status.valid == 1) {
            // Send an immediate update
            send_s(NVPtr->debugflags);
        } else {
            // Send something from the refresh stack
            send_q(NVPtr->debugflags, NVPtr->opflags.enableRailcomCutout);
        }
    } else {
        // Send an idle
        send_idle();
    }
}

//
// queue_add()
//
// Attempt to add an entry to the Q queue in response to OPC_RLOC
//
// The queue may only have one packet for a given decoder address, which
// must be greater than 0. If the address is already in use by a cab, or 
// the queue is full, then an error is returned. 
//
// If the address is in the queue but marked as dispatched, then that
// session is returned as the handle to the cab, otherwise the queue 
// entry is created and a handle is returned to the cab.
// 
// The queue index is the session handle.
//

// ??? Change so when re-using cached entries, count on through rather than keep re-using the same one.
// Returns session number

BYTE queue_add(WORD addr, enum glocModes requestmode, ModNVPtr cmdNVPtr) {

    BYTE i, free_handle, cached_handle, cache_reuse_handle, use_handle, err;

    // Ignore attempts to use address 0
    if (addr == 0) {
        return(0xFF);
    }
    // Find free entry or match address
    free_handle = 0;
    cached_handle = 0;
    cache_reuse_handle = 0;
    use_handle = 0xFF;  // set invalid session no. to start with

    i = 0;
    err = ERR_LOCO_STACK_FULL;

    while (i < MAX_HANDLES) {
        if (q_queue[i].status.valid == 0) {
            if ((free_handle == 0) && (q_queue[i].status.cached == 0)) {
                // Found first free entry, save it for later
                free_handle = i;
                err = 0;
            } else if (q_queue[i].status.cached == 1) {
                err = 0;
                if (q_queue[i].address.addr_int == addr) {
                    // Found cached data for this loco, save it for later
                    cached_handle = i;
                    err = 0;
                } else if (cache_reuse_handle == 0) {
                    // Found a cached entry, save it for later in case there are no completely free handles
                    // and we might have to re-use a cached one
                    cache_reuse_handle = i;
                    err = 0;
                }
            }
        }// if not a valid entry 
        else if (q_queue[i].address.addr_int == addr) {
            // Found same address in valid slot
            err = ERR_LOCO_ADDR_TAKEN;
            break;
        }
        i++;
    } // while i<max_handles

    if (err == ERR_LOCO_ADDR_TAKEN) {
        if (q_queue[i].status.dispatched == 1) // dispatched loco
        {
            q_queue[i].status.dispatched = 0;
            send_ploc(i);
            err = 0;

            // Temporary code for shuttle proof of concept
            if (q_queue[i].status.shuttle == 1) {
                clear_shuttle_entry(i);
                q_queue[i].status.shuttle = 0;
            }

        } else {
            switch (requestmode) {
                case glocSteal:
                    // issue forced release to current cab(s)
                    if (cmdNVPtr->userflags.PermitSteal) {
                        force_release(i, TRUE);
                        q_queue[i].status.share_count = 0;
                        // Now give it to the new cab
                        send_ploc(i);
                        err = 0;
                    }
                    break;

                case glocShare:
                    // Increment share count
                    if (cmdNVPtr->userflags.PermitShare) {
                        if (q_queue[i].status.share_count < 3)
                            q_queue[i].status.share_count++;
                        // Pass details to new sharer
                        send_ploc(i);
                        err = 0;
                    }
                    break;

                default:
                    break; // Do nothing for normal mode so taken error will be returned
            }


        }
    } else if (err == 0) {
        if (cached_handle != 0) {
            use_handle = cached_handle;
        } else if (free_handle != 0) {
            use_handle = free_handle;
        } else if (cache_reuse_handle != 0) {
            use_handle = cache_reuse_handle;
            purge_session(cache_reuse_handle);
        } else {
            err = ERR_LOCO_STACK_FULL; // shouldn't ever get here as err should have been zero for this block - but code defensively to allow for it
        }


        if (err == 0) {
            // use_handle is the handle of available entry
            // Initialise the entry
            q_queue[use_handle].address.addr_int = addr;
            // Ensure correct DCC address format
            q_queue[use_handle].address.addr_hi.long0 = q_queue[use_handle].address.addr_hi.long1;
            q_queue[use_handle].status.valid = 1;
            q_queue[use_handle].status.cached = 0;
            q_queue[use_handle].status.share_count = 0;
            q_queue[use_handle].status.dispatched = 0;
            send_ploc(use_handle);
        }
    }


    if (err != 0) {
        // Report error code
        Tx1[d0] = OPC_ERR;
        Tx1[d1] = rx_ptr->d1;
        Tx1[d2] = rx_ptr->d2;
        Tx1[d3] = err;
        sendCbusMsg();
    }
    return( use_handle);
} // queue add


//
// Send ploc
//
// Sends a PLOC opcode for the cab to give it the session id (handle) and also the current speed and
// funtion settings
//

void send_ploc(BYTE handle)
 {
    // Reset timeout whenever responding with handle (usually when loco allocated)
    keep_alive(handle);
    // Report to cab
    Tx1[d0] = OPC_PLOC;
    Tx1[d1] = handle;
    Tx1[d2] = q_queue[handle].address.addr_hi.byte;
    Tx1[d3] = q_queue[handle].address.addr_lo;
    Tx1[d4] = q_queue[handle].speed;
    Tx1[d5] = q_queue[handle].fn1.byte;
    Tx1[d6] = q_queue[handle].fn2.byte;
    Tx1[d7] = q_queue[handle].fn2a.byte;
    sendCbusMsg();

} // send ploc

//
// Force release
//
// Sends a session not found error message. This is sent to force release by the cab, either because it has sent
// a packet for a timed out session or because another cab is forcing take over
//

void force_release(BYTE handle, BOOL stolen)
 {
    Tx1[d0] = OPC_ERR;
    Tx1[d1] = handle;
    Tx1[d2] = 0;
    Tx1[d3] = (stolen ? ERR_SESSION_CANCELLED : ERR_SESSION_NOT_PRESENT);
    sendCbusMsg();
} // force release



//
// throttle_mode()
//
// Set the throttle speed step mode in response to OPC_STMOD. 128, 28 and 14
// speed steps are supported. 28 with interleaved steps will send 28 steps.
//
//	If msbit of mode byte is set, then ls 7 bits are the shuttle number to release
//	this loco into (no longer used as ALOC opcode now used instead)
//

void throttle_mode(void) {

    BYTE session, shuttle_id;

    session = rx_ptr->d1;

    if (q_queue[session].status.valid == 1) {
        // Handle is valid so update it's mode
        q_queue[session].status.throttle_mode = rx_ptr->d2 & TMOD_SPD_MASK; 
        
        
        
//        if ((rx_ptr->d2 && 0x80) == 0) {
//            // Handle is valid so update it's mode
//            
//        } else {
//            shuttle_id = rx_ptr->d2 & 0x80;
//            set_shuttle_loco(session, shuttle_id);
//            q_queue[session].status.shuttle = TRUE;
//        }
    } else {
        force_release(session, FALSE);
    }
}


//
// loco_function()
//
// Send a function change to the loco - session and function number passed as paramters
// 

void loco_function(enum funcops dofunc, BYTE session, BYTE funcnum) {

    BYTE funcrange;
    BYTE funcvalues;

    if (q_queue[session].status.valid == 1) {
        // Handle is valid so update function byte

        funcvalues = 1;
        
        if (funcnum > 28)
        {
            funcrange = 6;
            funcnum -= 29;
            funcvalues <<= funcnum;  // not enough bits!
        }    
            
        else if (funcnum > 20) {
            funcrange = 5;
            funcnum -= 21;
            funcvalues <<= funcnum;

            // ??? will need to track status of these in the stack so can set/reset the bits
        } else if (funcnum > 12) {
            funcrange = 4;
            funcnum -= 13;
            funcvalues <<= funcnum;

            // ??? will need to track status of these in the stack so can set/reset the bits
        } else if (funcnum > 8) {
            funcrange = 3;
            funcnum -= 9;
            funcvalues <<= funcnum;
            funcvalues = set_funcop(&(q_queue[session].fn2a.byte), dofunc, funcvalues);

        } else if (funcnum > 4) {
            funcrange = 2;
            funcnum -= 5;
            funcvalues <<= funcnum;
            funcvalues = set_funcop(&(q_queue[session].fn2.byte), dofunc, funcvalues);
        } else {
            funcrange = 1;
            if (funcnum == 0)
                funcnum = 5; // F0 (FL) is stored in bit 4

            funcvalues <<= funcnum - 1;
            funcvalues = set_funcop(&(q_queue[session].fn1.byte), dofunc, funcvalues);
        }
        queue_update(session, funcrange, funcvalues);
    } else {
        force_release(session, FALSE);
    }
} // loco function



// set_funcop applies the function operation (on/off/toggle) to the DCC function byte passed as a pointer. 
// The modified DCC function byte is returned
// Note - that means that this is a function with side effects

BYTE set_funcop(BYTE *funcbyte, enum funcops dofunc, BYTE funcvalues) {
    switch (dofunc) {
        case funcon: *funcbyte |= funcvalues; // Turn function on
            break;

        case funcoff: funcvalues = ~funcvalues;
            *funcbyte &= funcvalues; // Turn function off
            break;

        case functog: *funcbyte ^= funcvalues; // Toggle function 
            break;

    }
    return ( *funcbyte);

} // set_funcop

void speed_update(BYTE session, BYTE speed)
 {
    // Only update for valid slots
    if (q_queue[session].status.valid == 1) 
    {
        if ((speed & 0x7F) > cmdNVptr->maxSpeed)
            speed = (speed & 0x80) + cmdNVptr->maxSpeed;
        q_queue[session].speed = speed;
        queue_update(session, 0, 0);
        if ((speed & 0x7F) > 0)
            CSStatus.EMStopAllActive = FALSE;
    } else {
        force_release(session, FALSE);
    }

} // speed update

void function_update(BYTE session, BYTE funcrange, BYTE funcvalues)
 {
    // Only update for valid slots
    if (q_queue[session].status.valid == 1) {

        switch (funcrange) {
                // Fn0 - 12 go in refresh queue
            case 1:
                q_queue[session].fn1.byte = funcvalues;
                break;

            case 2:
                q_queue[session].fn2.byte = funcvalues;
                break;

            case 3:
                q_queue[session].fn2a.byte = funcvalues;
                break;

            default:
                break;
        }

        queue_update(session, funcrange, funcvalues);

    }// if valid session
    else {
        force_release(session, FALSE);
    }
} // function_update


//
// queue_update()
//
// Update speed/dir or function state in response to OPC_DSPD or OPC_DFUNx
// for handle and add an immediate speed/dir or function update to the S 
// queue.  Set funcrange to zero for a speed change, or the CBUS function mode
// for a function update
//

void queue_update(BYTE session, BYTE funcrange, BYTE funcvalues)
 {
    unsigned char i = 0;
    unsigned char speed;

    // get address of next s queue entry
    dcc_queue_t * s_ptr = &(s_queue[s_head]);

    // Only update for valid slots
    if (q_queue[session].status.valid == 1) {

        // Reset slot timeout 
        keep_alive(session);

        speed = q_queue[session].speed;

        // Add immediate packet to S with new speed if possible
        if (s_ptr->status.valid == 0) {
            s_ptr->d[5] = 0; // clear error detection byte
            if (q_queue[session].address.addr_hi.long1 == 1) {
                // Put long address in
                s_ptr->d[i] = q_queue[session].address.addr_hi.byte;
                s_ptr->d[5] = s_ptr->d[i++]; // Build error detection byte
                s_ptr->d[i] = q_queue[session].address.addr_lo;
                s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++]; // Error detection byte is xor of packet bytes
            } else {
                // Put short address in
                s_ptr->d[i] = q_queue[session].address.addr_lo;
                s_ptr->d[5] = s_ptr->d[i++];
            }
            if (funcrange == 0) {
                switch (q_queue[session].status.throttle_mode) {
                    case TMOD_SPD_128:
                        s_ptr->d[i] = 0x3F;
                        s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
                        // Speed byte
                        s_ptr->d[i] = speed;
                        break;

                    case TMOD_SPD_28:
                    case TMOD_SPD_28I:
                        // 28 speed steps
                        s_ptr->d[i] = step28(speed);
                        // s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
                        break;

                    case TMOD_SPD_14:
                        // 14 speed steps
                        // *** ??? Put Fn0 into speed byte
                        s_ptr->d[i] = step14(speed);
                        // s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
                        break;

                }
                s_ptr->repeat = 1; // Send speed once
            } else {
                // send function packet
                switch (funcrange) {
                    case 1:
                        // Fn0 - 4 
                        s_ptr->d[i] = 0x80 | (funcvalues & 0x1F);
                        break;

                    case 2:
                        // Fn5 - 8 
                        s_ptr->d[i] = 0xB0 | (funcvalues & 0x0F);
                        break;

                    case 3:
                        // Fn9 - 12 
                        s_ptr->d[i] = 0xA0 | (funcvalues & 0x0F);
                        break;

                    case 4:
                        // Fn13 - 20 
                        s_ptr->d[i] = 0xDE;
                        s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
                        s_ptr->d[i] = funcvalues;
                        break;

                    case 5:
                        // Fn21 - 28
                        s_ptr->d[i] = 0xDF;
                        s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
                        s_ptr->d[i] = funcvalues;
                        break;

                    default:
                        break;
                }
                s_ptr->repeat = 2; // Send Fn control twice
            }
            s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
            // Error detection byte
            s_ptr->d[i++] = s_ptr->d[5];
            s_ptr->status.byte_count = i;
            // Mark as valid and point to next entry
            s_ptr->status.valid = 1;
            s_head = (s_head + 1) & 0x0f;
        } else {
            // No rooom yet
            mode_word.s_full = 1;
        }
    } else {
        // Slot was invalid   
        force_release(session, FALSE);
    }
} // queue_update





//
// query_session
//
// Query loco by session number
//

void query_session(BYTE session) {

    if (q_queue[session].status.valid == 1) {
        // Valid session so respond with PLOC
        Tx1[d0] = OPC_PLOC;
        Tx1[d1] = session;
        Tx1[d2] = q_queue[session].address.addr_hi.byte;
        Tx1[d3] = q_queue[session].address.addr_lo;
        Tx1[d4] = q_queue[session].speed;
        Tx1[d5] = q_queue[session].fn1.byte;
        Tx1[d6] = q_queue[session].fn2.byte;
        Tx1[d7] = q_queue[session].fn2a.byte;
        sendCbusMsg();
    } else {
        // Invalid session so respond with error session not found
        // Can't give an address so send requesting handle in byte 1
        force_release(session, FALSE);
    }
} // query_session



//
// check_session_timeouts
//
// Check timeouts - stop train and release session if it times out
//

void check_session_timeouts(ModNVPtr NVPtr) {
    BYTE i;

    for (i = 0; i < MAX_HANDLES; i++) {
        if ((q_queue[i].status.valid) && (!q_queue[i].status.dispatched) && (!q_queue[i].status.shuttle)) {
            if (--q_queue[i].timeout == 0) {
                if (((q_queue[i].speed & 0x7F) == 0) || !NVPtr->userflags.stopontimeout) {
                    release_loco(i, FALSE); // Will dispatch if speed non-zero
                    force_release(i, FALSE); // Send session not found message
                } else // Speed non zero and stop on timeout flag set, so stop train first
                {
                    speed_update(i, q_queue[i].speed & 0x80); //Bring timed out train to a stop
                    q_queue[i].timeout = 1; // Tick one more time to allow stop command to go out
                }
            }
        }
    } // for i
} // check session timeouts


//
// Release loco
//
// If loco is moving, mark as dispatched, otherwise mark session as available (cache loco data)
//

void release_loco(BYTE session, BOOL shuttlesActive) {

    if (q_queue[session].status.valid == 1) 
    {
        if (q_queue[session].status.share_count > 0) {
            q_queue[session].status.share_count--;
        } else if (!q_queue[session].status.shuttle && ((q_queue[session].speed & 0x7f) <= 1)) {
            cache_session(session);
        } else 
        {
            q_queue[session].status.dispatched = 1;
            // Temporary code for shuttle proof of concept - populate first 3 shuttle entries with dispatched locos - no longer used since can now allocate directly into shuttle
//            if (shuttlesActive)
//                if (!populate_shuttle(session, 0, TRUE)) {
//                    if (!populate_shuttle(session, 1, TRUE)) {
//                        populate_shuttle(session, 2, TRUE);
//                }
//            }
        }
    } else 
    {
        force_release(session, FALSE);
    }

} // release loco



//
// purge_session
//
// Purge loco session from the queue
//

void purge_session(BYTE session) {
    // rx_ptr->d1 &= 0x1f;

    q_queue[session].status.byte = 0;
    q_queue[session].address.addr_int = 0;
    q_queue[session].speed = 0x80;
    q_queue[session].fn1.byte = 0;
    q_queue[session].fn2.byte = 0;
    q_queue[session].fn2a.byte = 0;
    q_queue[session].timeout = 0;
} // purge session

//
// cache_session
//
// Loco session is not longer valid in the queue
// but the data is kept cached whilst there is space in the queue
// for use when loco is next selected - so keeps consistent state of functions
//

void cache_session(BYTE session) {
    // rx_ptr->d1 &= 0x1f;

    q_queue[session].status.valid = 0;
    q_queue[session].status.dispatched = 0;
    q_queue[session].status.cached = 1;
} // cache session



//
// keep_alive()
//
// bump session timeout
//

void keep_alive(BYTE session) {

    if (q_queue[session].status.valid) {
        q_queue[session].timeout = nodevartable.module_nodevars.walktimeout * 2; // Count of half second heartbeats
    } else {
        force_release(session, FALSE);
    }
} // keep_alive

//
// dcc_packet()
//
// Send a DCC packet
//

void dcc_packet(void) {
    // get address of next s queue entry
    dcc_queue_t * s_ptr = &(s_queue[s_head]);
    // Add to next S queue entry if possible
    if (s_ptr->status.valid == 0) {
        switch (rx_ptr->d0) {
            case OPC_RDCC6:
                s_ptr->d[5] = rx_ptr->d7;
                // fall through
            case OPC_RDCC5:
                s_ptr->d[4] = rx_ptr->d6;
                // fall through
            case OPC_RDCC4:
                s_ptr->d[3] = rx_ptr->d5;
                // fall through
            case OPC_RDCC3:
                s_ptr->d[2] = rx_ptr->d4;
                s_ptr->d[1] = rx_ptr->d3;
                s_ptr->d[0] = rx_ptr->d2;
                s_ptr->repeat = rx_ptr->d1 & 0x7;
                break;

            default:
                break;
        }
        // Byte count is in OPC
        s_ptr->status.byte_count = ((rx_ptr->d0 & 0xE0) >> 5) - 1;
        // Mark as valid and point to next entry
        s_ptr->status.valid = 1;
        s_head = (s_head + 1) & 0x0f;
    } else {
        // No room yet
        mode_word.s_full = 1;
    }
}

//
// consist_add()
//
// Add ops mode programming commands to S queue to put loco in a 
// consist by writing to CV19. Consist address of zero will remove
// the loco from the consist
//

void consist_add(void) {
    dcc_address ops_address;

    // CV address for Consist Address
    cv = 19 - 1;
    // Consist address
    cv_data = rx_ptr->d2;
    ops_address = q_queue[rx_ptr->d1].address;
    ops_write(ops_address, cv, cv_data, 1);
}

//
// ops_write()
//
// Add an ops mode direct bit or byte write to the S queue if possible.
// The session number is used to look up the address in the Q queue.
//
// The packet is repeated twice
//

void ops_write(dcc_address ops_address, WORD cv_num, BYTE cv_data, BYTE write_mode) {
    unsigned char i = 0;

    // get address of next s queue entry
    dcc_queue_t * s_ptr = &(s_queue[s_head]);

    // Add to next S queue entry if possible
    if (s_ptr->status.valid == 0) {
        s_ptr->d[5] = 0; // clear error byte
        if (ops_address.addr_hi.long1 == 1) {
            // Put long address in
            s_ptr->d[i] = ops_address.addr_hi.byte;
            s_ptr->d[5] = s_ptr->d[i++];
            s_ptr->d[i] = ops_address.addr_lo;
            s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
        } else {
            // Put short address in
            s_ptr->d[i] = ops_address.addr_lo;
            s_ptr->d[5] = s_ptr->d[i++];
        }

        write_mode &= 0x03; // ops mode only supports direct byte or direct bit modes, so mask other bits in mode byte

        if (write_mode == 1) {
            // Instruction byte for CV access long form for direct byte
            s_ptr->d[i] = 0b11101100 | ((cv_num >> 8) & 0x3);
        } else {
            // Instruction byte for CV access long form for direct bit
            s_ptr->d[i] = 0b11101000 | ((cv_num >> 8) & 0x3);
        }
        s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
        // CV LSBs
        s_ptr->d[i] = cv_num & 0xFF;
        s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
        // Data will be data byte (WCVO or PCON) or bit manipulation instruction (WCVB)
        s_ptr->d[i] = cv_data;
        s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
        // Error byte
        s_ptr->d[i++] = s_ptr->d[5];
        s_ptr->status.byte_count = i;
        // Repeat
        s_ptr->repeat = 2;
        // Mark as valid and point to next entry
        s_ptr->status.valid = 1;
        s_head = (s_head + 1) & 0x0f;
    } else {
        // No room yet
        mode_word.s_full = 1;
    }
}

//
// dccAccessoryWrite()
//
// Add an accessory DCC packet to the S queue if possible.
//
// The 12 bit DCC accessory address is passed to this routine 
// (Note: any mapping from CBUS event to accessory address is done before calling this routine) 
// This can also be considered to be a 9 bit accessory address which is the CBUS event divided by 8, 
// with bit zero giving which of a pair to operate, and bits 1 and 2 giving which of 4 pairs of outputs to operate
// (which way this is viewed depends on the accessory decoder address numbering convention - the actual packet is the same either way))
// 
// The packet is repeated twice
//

void dccAccessoryWrite(WORD acc_num, BOOL accOn) {

    dcc_address acc_address;

    // get address of next s queue entry 
    dcc_queue_t * s_ptr = &(s_queue[s_head]);

    // Add to next S queue entry if possible
    if (s_ptr->status.valid == 0) {
        s_ptr->d[5] = 0; // clear error byte

        acc_address.addr_int = acc_num & 0x7FF; // Max accessory address is 12 bit, as LS bit will be our event on/off, we will use up to  11 bits of the accessory number
      
        // Work out accessory address from event number
        // From NMRA RP9.2.1 D - accessory decoder packet format
        // First  byte is 10AAAAAA where AAAAAA is bits 3 to 8 of the accessory address
        s_ptr->d[0] = 0x80 + ((((acc_address.addr_lo) >> 2) + 1) & 0x3F) + ((acc_address.addr_hi.byte << 5) & 0x20); 
        // Second byte is 1AAACDDD where DDD are bits 0 to 2 of the accessory address, C is 'activate' or 'deactivate' and AAA are the inverted bits 9 to 11 of the accessory address
        // For CBUS events, activate is always on, and the LSbit of the DCC address corresponds to accessory on or accessory off
        s_ptr->d[1] = 0x88 + (~(acc_address.addr_hi.byte << 4) & 0x70) + ((acc_address.addr_lo & 0x03)<<1 ); 
        
        if (accOn)
            s_ptr->d[1] |= 0x01; // Accessory on bit
             
        s_ptr->d[2] = s_ptr->d[0] ^ s_ptr->d[1];
        
        s_ptr->status.byte_count = 3;
        // Repeat count - send it once
        s_ptr->repeat = 1;
        // Mark as valid and point to next entry
        s_ptr->status.valid = 1;
        s_head = (s_head + 1) & 0x0f;
    } else {
        // No room yet
        mode_word.s_full = 1;
    }
}

BOOL dccAccessoryRoute(WORD event_num, BOOL accOn)
{
    AccRoutePtr        routePtr;
    BYTE               routeCounter;
    BOOL               routeFound = FALSE;
    BYTE               acc;
            
    
    if (accOn)  // Route only responds to ON event
    {
        routePtr = cmdNVptr->accRouteTable;

        routeCounter = 0;
        
        while (routeCounter < MAX_ROUTES)
        {    
            if (routeFound = (routePtr->mappedEvent == event_num))
            {    
                for (acc = 0; acc < ACCS_PER_ROUTE; acc++)
                {
                    if (routePtr->accessories[acc].accFlags.inUse)
                        dccAccessoryWrite(routePtr->accessories[acc].accAdress, routePtr->accessories[acc].accFlags.accON);
                }    
            }
            routeCounter++;
            routePtr++;
        }    
    }  
    return(routeFound);
}
    

void send_s(CancmdDbugFlags debugflags) {
    unsigned char i;

    // Send a packet from the S queue
    // number of bytes
    dcc_bytes_m = s_queue[s_tail].status.byte_count;
    dcc_buff_m[6] = 1; // send once
    // copy the packet
    for (i = 0; i < dcc_bytes_m; i++) {

        dcc_buff_m[i] = s_queue[s_tail].d[i];
    }
    if (s_queue[s_tail].repeat > 1) {
        // count repeats and force idle insertion between repetitions
        s_queue[s_tail].repeat = s_queue[s_tail].repeat - 1;
        idle_next = 1;
    } else {
        // done with this slot
        s_queue[s_tail].status.valid = 0;
        s_tail = (s_tail + 1) & 0x0f; // wrap at 16
    }
    // hand off buffer	
    dcc_flags.dcc_rdy_m = 0;


    // If debug flag set, send a CBUS packet containing the DCC packet we just started
    if (debugflags.prioritypackets) {
        sendCbusDebugEvent(Node_id, (BYTE *) & dcc_buff_m);
    }
}


// Many decoders will not send their Railcom data in the cutout when only idle packets are being transmitted. If no locos are in the stack, and Railcom is enabled, use a speed 0 to loco short address 127 as a pseudo idle message.

void sendPseudoIdle(void) 

{
    unsigned char i = 0;

    // get address of next s queue entry
    dcc_queue_t * s_ptr = &(s_queue[s_head]);

    // Add immediate packet to S with zero speed for short address 127

    if (s_ptr->status.valid == 0) 
    {
        s_ptr->d[5] = 0; // clear error detection byte
        // Put short address in
        s_ptr->d[i] = 127;  // short loco address
        s_ptr->d[5] = s_ptr->d[i++];
        s_ptr->d[i] = 0x3F; // 128 step mode
        s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
        s_ptr->d[i] = 0; // Zero speed 
        s_ptr->d[5] = s_ptr->d[5] ^ s_ptr->d[i++];
        s_ptr->d[i++] = s_ptr->d[5];
        s_ptr->status.byte_count = i;
        s_ptr->repeat = 1; // Send packet once
        // Mark as valid and point to next entry
        s_ptr->status.valid = 1;
        s_head = (s_head + 1) & 0x0f;
    }
}


void send_q(CancmdDbugFlags debugflags, BOOL railcomEnabled) {
    unsigned char i;
    unsigned char speed;
    BOOL    foundLoco;

    // send a packet from the Q queue
    if (q_queue[q_idx].status.valid == 1) {
        // Assemble the packet
        i = 0;
        if (q_queue[q_idx].address.addr_hi.long1 == 1) {
            // long address
            dcc_buff_m[i] = q_queue[q_idx].address.addr_hi.byte;
            dcc_buff_m[5] = dcc_buff_m[i++];
            dcc_buff_m[i] = q_queue[q_idx].address.addr_lo;
            dcc_buff_m[5] = dcc_buff_m[5] ^ dcc_buff_m[i++];
        } else {
            // short address
            dcc_buff_m[i] = q_queue[q_idx].address.addr_lo;
            dcc_buff_m[5] = dcc_buff_m[i++];
        }
        switch (q_state) {
            case (0):
                // Function group 1
                dcc_buff_m[i] = 0x80 | q_queue[q_idx].fn1.byte;
                break;

            case (1):
                // Function group 2
                dcc_buff_m[i] = 0xB0 | q_queue[q_idx].fn2.byte;
                break;

            case (2):
                // Function group 2a
                dcc_buff_m[i] = 0xA0 | q_queue[q_idx].fn2a.byte;
                break;

            default:
                // Send a speed update
                speed = q_queue[q_idx].speed;
                switch (q_queue[q_idx].status.throttle_mode) {
                    case TMOD_SPD_128:
                        // 128 speed steps
                        dcc_buff_m[i] = 0x3F;
                        dcc_buff_m[5] = dcc_buff_m[5] ^ dcc_buff_m[i++];
                        // Speed byte
                        dcc_buff_m[i] = speed;
                        break;

                    case TMOD_SPD_28:
                    case TMOD_SPD_28I:
                        // 28 speed steps
                        speed = step28(speed);
                        dcc_buff_m[i] = speed;
                        break;

                    case TMOD_SPD_14:
                        // 14 speed steps
                        speed = step14(speed);
                        // *** Put Fn0 into speed byte
                        dcc_buff_m[i] = speed;
                        break;

                }
                break;

        } // switch q_state

        dcc_buff_m[5] = dcc_buff_m[5] ^ dcc_buff_m[i++];
        dcc_buff_m[i++] = dcc_buff_m[5]; // final error byte
        dcc_bytes_m = i;
        dcc_buff_m[6] = 1; // send once

        // hand off buffer
        dcc_flags.dcc_rdy_m = 0;


        // If debug flag set, send a CBUS packet containing the DCC packet we just started

        if (((q_state > 2) && (debugflags.refreshspeedpackets))
                || ((q_state <= 2) && (debugflags.refreshfuncpackets))) {
            sendCbusDebugEvent(Node_id, (BYTE *) & dcc_buff_m);
        }
    } else {
        // Not valid so send an idle immediately
        send_idle();
    }

    // Find next valid slot
    i = MAX_HANDLES;
    foundLoco = FALSE;
    
    while (i > 0) {
        q_idx++;
        if (q_idx >= MAX_HANDLES) {
            // Wrap around, send an idle next time to prevent
            // back to back packets if only one loco in stack
            q_idx = 0;
            idle_next = 1;
            q_state++;
            if (q_state > 6) {
                // Cycles through 7 states
                // 4 speed refresh, 3 function refresh
                q_state = 0;
            }
        }
        i--;
        if (q_queue[q_idx].status.valid == 1) {
            // Found one
            foundLoco = TRUE;
            i = 0;
        }
    }

    if ((foundLoco == FALSE) && railcomEnabled)
        sendPseudoIdle();
        
        
    // Find next valid slot
    //	q_idx++;
    //    i = MAX_HANDLES;
    //    while ((i > 0) && (q_queue[q_idx].status.valid == 0)) {
    //        if (q_idx >= MAX_HANDLES) {
    //            // Wrap around, send an idle next time to prevent
    //            // back to back packets if only one loco in stack
    //            q_idx = 0;
    //            idle_next = 1;
    //            q_state++;
    //            if (q_state > 6) {
    //                // Cycles through 7 states
    //                // 4 speed refresh, 3 function refresh
    //                q_state = 0;
    //            }
    //        }
    //        i--;
    //    }
    //    // Point to next slot
    //    q_idx++;
    //    if (q_idx >= MAX_HANDLES) {
    //        // Wrap around
    //        q_idx = 0;
    //        q_state++;
    //        if (q_state > 6) {
    //            // Cycles through 7 states
    //            // 4 speed refresh, 3 function refresh
    //            q_state = 0;
    //        }
    //    }
}

void send_idle(void) {
    
    // send an idle packet
    dcc_buff_m[0] = 0xff;
    dcc_buff_m[1] = 0;
    dcc_buff_m[2] = 0xff;
    dcc_bytes_m = 3;
    dcc_buff_m[6] = 1; // send honk
    // hand off buffer
    dcc_flags.dcc_rdy_m = 0;
    idle_next = 0;
}

