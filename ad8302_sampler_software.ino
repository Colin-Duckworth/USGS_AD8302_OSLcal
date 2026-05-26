// AD8302 gain/phase sampler — Arduino Uno (ATmega328P)
// VMAG → A0   VPHS → A1
// UART: 115200 baud, 8N1

#include "ad8302.h"

// *NOTE on MODES* 
// Mode 0 -> Sends data to RFsoc via UART
// Mode 1 -> Just prints gain/phase data measured to serial monitor & OLED
// Mode 2 -> Runs OSL protocol to calibrate the system by getting 3 term error array in EEPROM

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup(void)
{
    Serial.begin(115200);
    Serial.println(F("DBG: serial up"));

    OLED_startup();
    Serial.println(F("DBG: OLED done"));

    pinMode(BTN_PIN, INPUT);
    adcVrefMv = measureAref();
    Serial.println(F("DBG: aref done"));

#if MODE == 1
    setup_print_mode1();
#elif MODE == 2
    Serial.println(F("DBG: generate_freq_points"));
    generate_freq_points();
    Serial.println(F("DBG: OSL_calibration_initiate"));
    OSL_calibration_initiate();
    Serial.println(F("DBG: compute_error_terms"));
    compute_error_terms();
    Serial.println(F("DBG: done"));
#endif
}

void loop(void)
{
    // ── Periodic AREF update ─────────────────────────────────────────────────
    static unsigned long lastVrefMs = 0;
    unsigned long now = millis();
    if (now - lastVrefMs >= VREF_UPDATE_MS) {
        lastVrefMs = now;
        adcVrefMv  = measureAref();
    }

#if MODE == 0
    if (Serial.available() > 0) {
        uint8_t cmd = (uint8_t)Serial.read();
        if (cmd == 'S') {
            sampleAndSend();
        }
    }

#elif MODE == 1
    static unsigned long lastMs = 0;
    if (now - lastMs >= POLL_INTERVAL_MS) {
        lastMs = now;
        sampleAndPrint();
    }
#endif
}