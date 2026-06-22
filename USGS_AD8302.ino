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
    STATE_OSL,          // collecting SHORT → OPEN → LOAD autonomously
    STATE_GAMMA_DELAY,  // first sweep: SPDT on delay line
    STATE_GAMMA_THRU    // second sweep: SPDT on thru path
};

static State state    = STATE_IDLE;
static int   freq_idx = 0;
static int   osl_std  = 0;   // 0=SHORT, 1=OPEN, 2=LOAD

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

        // ── OSL calibration ───────────────────────────────────────────────────
        case CMD_OSL_BEGIN:
            state    = STATE_OSL;
            osl_std  = 0;
            freq_idx = 0;
            switch2_short();
            comms_sendACK();
            break;

        // ── Gamma characterization ────────────────────────────────────────────
        case CMD_GAMMA_BEGIN:
            osl_load_from_eeprom();
            switch2_antenna();
            switch1_delay();
            state    = STATE_GAMMA_DELAY;
            freq_idx = 0;
            comms_sendACK();
            break;

        // ── Per-frequency measurement ─────────────────────────────────────────
        case CMD_FREQ: {
            float gain_dB, phase_deg, vmag_mV, vphs_mV;

            if (state == STATE_OSL)
            {
                if (freq_idx >= N_CAL_POINTS) { comms_sendNACK(); break; }
                osl_store_sample(osl_std, freq_idx++, freq_hz);
                comms_sendACK();

                if (freq_idx == N_CAL_POINTS) {
                    freq_idx = 0;
                    osl_std++;
                    if      (osl_std == 1) switch2_open();
                    else if (osl_std == 2) switch2_load();
                    else {
                        // all three standards collected
                        osl_compute_error_terms();
                        comms_sendCalDone();
                        state = STATE_IDLE;
                    }
                }
            }
            else if (state == STATE_GAMMA_DELAY)
            {
                if (freq_idx >= N_SWEEP_POINTS) { comms_sendNACK(); break; }
                delayMicroseconds(DELAY_AD8302_US);
                sample(&gain_dB, &phase_deg, &vmag_mV, &vphs_mV);
                gamma_delay_raw[freq_idx].gain_dB   = gain_dB;
                gamma_delay_raw[freq_idx].phase_deg = phase_deg;
                freq_idx++;
                comms_sendACK();

                if (freq_idx == N_SWEEP_POINTS) {
                    switch1_thru();
                    state    = STATE_GAMMA_THRU;
                    freq_idx = 0;
                }
            }
            else if (state == STATE_GAMMA_THRU)
            {
                if (freq_idx >= N_SWEEP_POINTS) { comms_sendNACK(); break; }
                delayMicroseconds(DELAY_AD8302_US);
                sample(&gain_dB, &phase_deg, &vmag_mV, &vphs_mV);
                gamma_thru_raw[freq_idx].gain_dB   = gain_dB;
                gamma_thru_raw[freq_idx].phase_deg = phase_deg;
                freq_idx++;
                comms_sendACK();

                if (freq_idx == N_SWEEP_POINTS) {
                    osl_apply_corrections();
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

        // ── Debug ─────────────────────────────────────────────────────────────
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
