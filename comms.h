#pragma once
#include "config.h"

// Commands received from RFSoC (newline-terminated ASCII).
//
//   OSL_BEGIN       — initiate OSL calibration; Arduino drives SHORT→OPEN→LOAD autonomously
//   FREQ <hz>       — RFSoC has tuned to this frequency; take a measurement
//   GAMMA_BEGIN     — load error terms; route SP6T to antenna, SPDT to delay; prepare sweep
//   DUMP_CAL        — dump error terms to DEBUG_SERIAL (bench debug only)
//
// Responses from Arduino to RFSoC:
//   ACK             — ready / step complete
//   NACK            — out-of-sequence or index overflow
//   CAL_DONE        — OSL complete; error terms saved to EEPROM
//   DATA_BEGIN      — start of gamma dataset
//   <hz>,<mag>,<ph> — one corrected GammaPoint (561 lines)
//   DATA_END        — end of gamma dataset

typedef enum {
    CMD_NONE,
    CMD_OSL_BEGIN,
    CMD_GAMMA_BEGIN,
    CMD_FREQ,
    CMD_DUMP_CAL,
    CMD_UNKNOWN
} Command;

bool    comms_readLine(char *buf, uint8_t maxLen);
Command comms_parseCommand(const char *buf, double *freq_out);
void    comms_sendACK(void);
void    comms_sendNACK(void);
void    comms_sendCalDone(void);
void    comms_sendSweepData(void);
