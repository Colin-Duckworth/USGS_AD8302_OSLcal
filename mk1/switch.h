#pragma once

void switch_init(void);

// SPDT — selects direct or delay-line path for phase ambiguity resolution
void switch1_thru(void);     // direct path  (pin2=HIGH, pin3=LOW)
void switch1_delay(void);    // delay-line path (pin2=LOW, pin3=HIGH)

// SP6T — selects which port is routed into the reflectometer signal path
void switch2_antenna(void);  // RF1: antenna (measurement mode)
void switch2_open(void);     // RF2: OPEN calibration standard
void switch2_short(void);    // RF3: SHORT calibration standard
void switch2_load(void);     // RF4: LOAD (50 Ω) calibration standard
// RF5, RF6: terminated, unused
