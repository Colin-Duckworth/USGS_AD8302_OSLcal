#pragma once
#include "config.h"

void adc_init(void);
void sample(float *gain_dB, float *phase_deg, float *vmag_mV, float *vphs_mV);
void gainphase_to_gamma(float gain_dB, float phase_deg, float *gamma_mag, float *gamma_phase_deg);
