#include "comms.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

bool comms_readLine(char *buf, uint8_t maxLen)
{
    // Static accumulator so partial lines survive across loop() calls.
    static char    rxBuf[32];
    static uint8_t rxIdx = 0;

    while (RFSOC_SERIAL.available()) {
        char c = (char)RFSOC_SERIAL.read();
        if (c == '\r') continue;
        if (c == '\n') {
            rxBuf[rxIdx] = '\0';
            memcpy(buf, rxBuf, rxIdx + 1);
            uint8_t len = rxIdx;
            rxIdx = 0;
            return len > 0;
        }
        if (rxIdx < maxLen - 1) rxBuf[rxIdx++] = c;
    }
    return false;
}

Command comms_parseCommand(const char *buf, double *freq_out)
{
    if (strcmp(buf, "OSL_OPEN")    == 0) return CMD_OSL_OPEN;
    if (strcmp(buf, "OSL_SHORT")   == 0) return CMD_OSL_SHORT;
    if (strcmp(buf, "OSL_LOAD")    == 0) return CMD_OSL_LOAD;
    if (strcmp(buf, "OSL_COMPUTE") == 0) return CMD_OSL_COMPUTE;
    if (strcmp(buf, "GAMMA_BEGIN") == 0) return CMD_GAMMA_BEGIN;
    if (strcmp(buf, "DUMP_CAL")    == 0) return CMD_DUMP_CAL;
    if (strncmp(buf, "FREQ ", 5)   == 0) {
        *freq_out = atof(buf + 5);
        return CMD_FREQ;
    }
    return CMD_UNKNOWN;
}

void comms_sendACK(void)
{
    RFSOC_SERIAL.println(F("ACK"));
}

void comms_sendNACK(void)
{
    RFSOC_SERIAL.println(F("NACK"));
}

void comms_sendSweepData(void)
{
    RFSOC_SERIAL.println(F("DATA_BEGIN"));
    for (int i = 0; i < N_SWEEP_POINTS; i++) {
        RFSOC_SERIAL.print((long)sweep_results[i].frequency_hz);
        RFSOC_SERIAL.print(',');
        RFSOC_SERIAL.print(sweep_results[i].gamma_mag, 6);
        RFSOC_SERIAL.print(',');
        RFSOC_SERIAL.println(sweep_results[i].gamma_phase_deg, 4);
    }
    RFSOC_SERIAL.println(F("DATA_END"));
}
