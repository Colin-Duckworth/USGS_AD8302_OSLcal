#include "switch.h"
#include "config.h"
#include <Arduino.h>

void switch_init(void)
{
    pinMode(SPDT_A_PIN, OUTPUT);
    pinMode(SPDT_B_PIN, OUTPUT);
    pinMode(SP6T_C1_PIN, OUTPUT);
    pinMode(SP6T_C2_PIN, OUTPUT);
    pinMode(SP6T_C3_PIN, OUTPUT);

    // Safe default: thru path, antenna port
    switch1_thru();
    switch2_antenna();
}

// ── SPDT ──────────────────────────────────────────────────────────────────────

void switch1_thru(void)
{
    digitalWrite(SPDT_A_PIN, HIGH);
    digitalWrite(SPDT_B_PIN, LOW);
    delayMicroseconds(DELAY_SPDT_US);
}

void switch1_delay(void)
{
    digitalWrite(SPDT_A_PIN, LOW);
    digitalWrite(SPDT_B_PIN, HIGH);
    delayMicroseconds(DELAY_SPDT_US);
}

// ── SP6T ──────────────────────────────────────────────────────────────────────

static void sp6t_set(uint8_t c1, uint8_t c2, uint8_t c3)
{
    digitalWrite(SP6T_C1_PIN, c1);
    digitalWrite(SP6T_C2_PIN, c2);
    digitalWrite(SP6T_C3_PIN, c3);
    delayMicroseconds(DELAY_SP6T_US);
}

void switch2_antenna(void) { sp6t_set(LOW,  LOW,  LOW);  }  // RF1
void switch2_open(void)    { sp6t_set(LOW,  LOW,  HIGH); }  // RF2
void switch2_short(void)   { sp6t_set(LOW,  HIGH, LOW);  }  // RF3
void switch2_load(void)    { sp6t_set(LOW,  HIGH, HIGH); }  // RF4
