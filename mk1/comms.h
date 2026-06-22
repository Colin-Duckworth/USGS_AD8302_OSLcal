#pragma once
#include "config.h"

// Commands the RFSoC sends to the Arduino (newline-terminated ASCII strings).
//
//   OSL_OPEN        — RFSoC has set up for OPEN standard; begin collecting
//   OSL_SHORT       — begin collecting SHORT standard
//   OSL_LOAD        — begin collecting LOAD standard
//   OSL_COMPUTE     — all standards collected; compute & save error terms
//   FREQ <hz>       — RFSoC has tuned to this frequency; take a measurement
//   SWEEP_BEGIN     — load error terms from EEPROM; prepare for sweep
//   DUMP_CAL        — dump error terms to DEBUG_SERIAL (bench debug only)
//
// Responses from Arduino to RFSoC:
//   ACK             — ready / step complete
//   NACK            — out-of-sequence or index overflow
//   DATA_BEGIN      — start of sweep dataset
//   <hz>,<mag>,<ph> — one corrected GammaPoint (561 lines)
//   DATA_END        — end of sweep dataset

typedef enum {
    CMD_NONE,
    CMD_OSL_OPEN,
    CMD_OSL_SHORT,
    CMD_OSL_LOAD,
    CMD_OSL_COMPUTE,
    CMD_GAMMA_BEGIN,
    CMD_FREQ,
    CMD_DUMP_CAL,
    CMD_UNKNOWN
} Command;

// Returns true and fills buf when a complete newline-terminated line is available.
// Non-blocking; accumulates across loop() calls.
bool    comms_readLine(char *buf, uint8_t maxLen);

Command comms_parseCommand(const char *buf, double *freq_out);
void    comms_sendACK(void);
void    comms_sendNACK(void);

// Streams sweep_results[N_SWEEP_POINTS] to RFSOC_SERIAL in CSV format.
void    comms_sendSweepData(void);
