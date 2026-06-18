// AD8302 reflectometer — Arduino UNO R4 Minima
// RFSoC is master; Arduino is slave.
// All communication via UART on Serial1 (D0/D1). See comms.h for protocol.

#include "config.h"
#include "adc.h"
#include "osl.h"
#include "comms.h"
#include "switch.h"

// ── State machine ─────────────────────────────────────────────────────────────
enum State {
    STATE_IDLE,
    STATE_OSL_OPEN,    // std_idx = 0
    STATE_OSL_SHORT,   // std_idx = 1
    STATE_OSL_LOAD,    // std_idx = 2
    STATE_GAMMA
};

static State state    = STATE_IDLE;
static int   freq_idx = 0;

// ── Entry points ──────────────────────────────────────────────────────────────
void setup(void)
{
    DEBUG_SERIAL.begin(UART_BAUD);
    RFSOC_SERIAL.begin(UART_BAUD);
    adc_init();
    switch_init();
    DEBUG_SERIAL.println(F("AD8302 reflectometer ready."));
}

void loop(void)
{
    char buf[32];
    if (!comms_readLine(buf, sizeof(buf))) return;

    double  freq_hz = 0.0;
    Command cmd     = comms_parseCommand(buf, &freq_hz);

    switch (cmd) {

        case CMD_OSL_OPEN:
            state = STATE_OSL_OPEN;
            freq_idx = 0;
            switch2_open();
            comms_sendACK();
            break;

        case CMD_OSL_SHORT:
            state = STATE_OSL_SHORT;
            freq_idx = 0;
            switch2_short();
            comms_sendACK();
            break;

        case CMD_OSL_LOAD:
            state = STATE_OSL_LOAD;
            freq_idx = 0;
            switch2_load();
            comms_sendACK();
            break;

        case CMD_OSL_COMPUTE:
            osl_compute_error_terms();
            state = STATE_IDLE;
            comms_sendACK();
            break;

        case CMD_GAMMA_BEGIN:
            osl_load_from_eeprom();
            switch2_antenna();
            state    = STATE_GAMMA;
            freq_idx = 0;
            comms_sendACK();
            break;

        case CMD_FREQ: {
            if (state == STATE_OSL_OPEN  ||
                state == STATE_OSL_SHORT ||
                state == STATE_OSL_LOAD)
            {
                if (freq_idx >= N_CAL_POINTS) { comms_sendNACK(); break; }
                int std_idx = state - STATE_OSL_OPEN;   // 0, 1, or 2
                osl_store_sample(std_idx, freq_idx++, freq_hz);
                comms_sendACK();
            }
            else if (state == STATE_GAMMA)
            {
                if (freq_idx >= N_SWEEP_POINTS) { comms_sendNACK(); break; }
                float gain_dB, phase_deg, vmag_mV, vphs_mV;
                sample(&gain_dB, &phase_deg, &vmag_mV, &vphs_mV);
                sweep_results[freq_idx++] = osl_correct(freq_hz, gain_dB, phase_deg);
                comms_sendACK();
                if (freq_idx == N_SWEEP_POINTS) {
                    comms_sendSweepData();
                    state    = STATE_IDLE;
                    freq_idx = 0;
                }
            }
            else
            {
                comms_sendNACK();
            }
            break;
        }

        case CMD_DUMP_CAL:
            osl_load_from_eeprom();
            osl_dump();
            comms_sendACK();
            break;

        default:
            comms_sendNACK();
            break;
    }
}
