


// Modified from Microchip standard version for different load address due to boot loader

/* Copyright (c)1999 Microchip Technology */

/* MPLAB-C18 startup code */

/* external reference to __init() function */
extern void __init (void);
/* external reference to the user's main routine */
extern void main (void);
/* prototype for the startup function */
void _entry (void);
void _startup (void);
extern near char __FPFLAGS;
#define RND 6

//#pragma code _entry_scn=0x000000
#pragma code _entry_scn=0x000800
/* Redirected reset vector comes here */
void
//_entry (void)
RESET_VECT (void)
{
_asm nop _endasm
_asm goto _startup _endasm

}

// #pragma code _startup_scn
#pragma code APP

void
_startup (void)
{
  _asm
    // Fill stack with known value for code profiling
    lfsr 1, 0x200
    movlw   0x80
    movwf   0, 0
    movlw   0xa5
fill_loop:    movwf   POSTINC1, 0
    decfsz  0, 1, 0
    bra     fill_loop
    
    // Initialize the stack pointer
    lfsr 1, _stack
    lfsr 2, _stack

    clrf TBLPTRU, 0 // 1st silicon doesn't do this on POR

    bcf __FPFLAGS,RND,0 // Initalize rounding flag for floating point libs
  _endasm loop:

  // If user defined __init is not found, the one in clib.lib will be used
  __init ();

  // Call the user's main routine
  main ();

  goto loop;
}                               /* end _startup() */
