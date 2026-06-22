#pragma once
#include "config.h"

// Store one OSL measurement into the transient calibration buffer.
// std_idx: 0=SHORT, 1=OPEN, 2=LOAD
void       osl_store_sample(int std_idx, int freq_idx, double freq_hz);

// Compute 3-term error model from buffered OSL measurements and save to EEPROM.
void       osl_compute_error_terms(void);

// Load error terms from EEPROM into error_terms[].
void       osl_load_from_eeprom(void);

// Resolve phase ambiguity across gamma_delay_raw/gamma_thru_raw,
// apply OSL correction, and write results into sweep_results[].
// TODO: implement once new delay line constant is characterised.
void       osl_apply_corrections(void);

// Dump error_terms[] to DEBUG_SERIAL as CSV (bench debug only).
void       osl_dump(void);
