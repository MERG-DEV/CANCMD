#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

typedef enum {FALSE = 0, TRUE = !0} BOOL;
typedef uint8_t BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef enum {INVALID = 0, VALID = 1} packet_status_t;

struct
{
  BYTE RB0;
  BYTE RB1;
} PORTBbits;

#define CANCMD 1
#define near
#define rom

#include "packet_gen.c"

DccFlags dcc_flags;
OpFlags  op_flags;

BYTE dcc_buff_s[7];
BYTE dcc_buff_m[7];
BYTE dcc_bytes_m;
BYTE Tx1[14];
WORD Node_id;

ecan_rx_buffer *rx_ptr;

ModNVPtr cmdNVptr;
ModeWord mode_word;

const NodevarTable nodevartable;

unsigned int  cv;
unsigned char cv_data;

ActiveShuttleEntry activeShuttleTable[MAX_HANDLES];

DelayListEntry delayedEvents[MAX_DELAYED_EVENTS];

void
clear_shuttle_entry(BYTE)
{}

void
sendCbusMsg()
{}

void
set_shuttle_loco( BYTE, BYTE)
{}

BOOL
populate_shuttle(BYTE session, BYTE shuttle_id, BOOL ifempty)
{
  return TRUE;
}

void
sendCbusDebugEvent(WORD, BYTE*)
{}

BOOL
addDelayedEvent(BYTE, BYTE, enum eventActions, BYTE)
{}

void
stopAll()
{}

int
main(int, char**)
{
  struct test_values
  {
    WORD event_number;
    BOOL is_acc_on;
    packet_status_t expected_status;
    BYTE expected_d0;
    BYTE expected_d1;
  };

  const BOOL ASON = TRUE;
  const BOOL ASOF = FALSE;

  const struct test_values test_data[] = {{   0, ASOF,   VALID, 0x81, 0xF8},
                                          {   0, ASON,   VALID, 0x81, 0xF9},
                                          {   1, ASOF,   VALID, 0x81, 0xFA},
                                          {   1, ASON,   VALID, 0x81, 0xFB},
                                          {   3, ASOF,   VALID, 0x81, 0xFE},
                                          {   3, ASON,   VALID, 0x81, 0xFF},
                                          {   4, ASOF,   VALID, 0x82, 0xF8},
                                          {   4, ASON,   VALID, 0x82, 0xF9},
                                          {   5, ASOF,   VALID, 0x82, 0xFA},
                                          {   5, ASON,   VALID, 0x82, 0xFB},
                                          { 250, ASOF,   VALID, 0xBF, 0xFC},
                                          { 250, ASON,   VALID, 0xBF, 0xFD},
                                          { 251, ASOF,   VALID, 0xBF, 0xFE},
                                          { 251, ASON,   VALID, 0xBF, 0xFF},
                                          { 252, ASOF,   VALID, 0x80, 0xE8},
                                          { 252, ASON,   VALID, 0x80, 0xE9},
                                          { 253, ASOF,   VALID, 0x80, 0xEA},
                                          { 253, ASON,   VALID, 0x80, 0xEB},
                                          { 254, ASOF,   VALID, 0x80, 0xEC},
                                          { 254, ASON,   VALID, 0x80, 0xED},
                                          { 255, ASOF,   VALID, 0x80, 0xEE},
                                          { 255, ASON,   VALID, 0x80, 0xEF},
                                          { 504, ASOF,   VALID, 0xBF, 0xE8},
                                          { 504, ASON,   VALID, 0xBF, 0xE9},
                                          {2039, ASOF,   VALID, 0xBE, 0x8E},
                                          {2039, ASON,   VALID, 0xBE, 0x8F},
                                          {2040, ASOF,   VALID, 0xBF, 0x88},
                                          {2040, ASON,   VALID, 0xBF, 0x89},
                                          {2041, ASOF,   VALID, 0xBF, 0x8A},
                                          {2041, ASON,   VALID, 0xBF, 0x8B},
                                          {2043, ASOF,   VALID, 0xBF, 0x8E},
                                          {2043, ASON,   VALID, 0xBF, 0x8F},
                                          {2044, ASOF,   VALID, 0x80, 0xF8},
                                          {2044, ASON,   VALID, 0x80, 0xF9},
                                          {2045, ASOF,   VALID, 0x80, 0xFA},
                                          {2045, ASON,   VALID, 0x80, 0xFB},
                                          {2046, ASOF,   VALID, 0x80, 0xFC},
                                          {2046, ASON,   VALID, 0x80, 0xFD},
                                          {2047, ASOF,   VALID, 0x80, 0xFE},
                                          {2047, ASON,   VALID, 0x80, 0xFF},
                                          {2048, ASOF,   VALID, 0x81, 0xF8},
                                          {2048, ASON,   VALID, 0x81, 0xF9}};
  const size_t number_of_tests = sizeof(test_data)/sizeof(test_data[0]);

  dcc_queue_t *s_ptr = NULL;

  s_head = 0;

  for (size_t test_index = 0; number_of_tests > test_index; ++test_index)
  {
    const struct test_values *test = &test_data[test_index];

    s_ptr = &s_queue[s_head];

    s_ptr->status.valid = INVALID;

    dccAccessoryWrite(test->event_number, test->is_acc_on);

    printf("Test %lu: Event %s(%u); ",
           test_index, test->is_acc_on ? "ASON" : "ASOF",
           test->event_number);
    if (INVALID == test->expected_status)
    {
      printf("Expected invalid packet; Got %s packet; %s\n",
             (INVALID == s_ptr->status.valid) ? "valid" : "invalid",
             (INVALID == s_ptr->status.valid) ? "PASS" : "FAIL");
    }
    else
    {
      printf("Expected: 0x%X, 0x%X; Got", test->expected_d0, test->expected_d1);
      if (INVALID == test->expected_status)
      {
        printf(" invalid packet; FAIL\n");
      }
      else
      {
        printf(": 0x%X, 0x%X; %s\n",
               s_ptr->d[0], s_ptr->d[1],
               (test->expected_d0 == s_ptr->d[0] &&
                test->expected_d1 == s_ptr->d[1]) ? "PASS" : "FAIL");
      }
    }
  }

  return 0;
}
