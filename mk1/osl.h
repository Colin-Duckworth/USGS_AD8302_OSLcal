#pragma once
#include "config.h"

// Store one OSL measurement into the transient calibration buffer.
// std_idx: 0=OPEN, 1=SHORT, 2=LOAD
void      osl_store_sample(int std_idx, int freq_idx, double freq_hz);

// Compute 3-term error model from buffered OSL measurements and save to EEPROM.
void      osl_compute_error_terms(void);

// Load error terms from EEPROM into error_terms[].
void      osl_load_from_eeprom(void);

// Apply OSL correction to a raw gain/phase measurement at freq_hz.
GammaPoint osl_correct(double freq_hz, float gain_dB, float phase_deg);

// Dump error_terms[] to DEBUG_SERIAL as CSV (bench debug only).
void      osl_dump(void);
