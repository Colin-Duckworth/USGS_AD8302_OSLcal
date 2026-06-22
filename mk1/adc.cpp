#include "adc.h"
#include <Arduino.h>

static inline float adcToMv(int counts)
{
    return counts * (AREF_MV / ADC_RESOLUTION);
}

void adc_init(void)
{
    // Use the AD8302 eval board's ~2.4 V AREF pin as the ADC reference.
    // AR_EXTERNAL is the RA4M1 Arduino core constant for the AREF pin.
    analogReference(EXTERNAL);
    // Flush mux — two dummy reads required after a reference change on RA4M1.
    analogRead(VMAG_PIN);
    analogRead(VMAG_PIN);
}

void sample(float *gain_dB, float *phase_deg, float *vmag_mV, float *vphs_mV)
{
    uint16_t magBuf[N_SAMPLES], phsBuf[N_SAMPLES];

    // Two settle reads before the burst to clear any mux residual.
    analogRead(VMAG_PIN);
    analogRead(VMAG_PIN);

    for (uint8_t i = 0; i < N_SAMPLES; i++) {
        magBuf[i] = analogRead(VMAG_PIN);
        analogRead(VPHS_PIN);           // dummy settle before phase read
        phsBuf[i] = analogRead(VPHS_PIN);
        analogRead(VMAG_PIN);           // dummy settle before next mag read
    }

    // Insertion sort — ascending, in-place
    for (uint8_t i = 1; i < N_SAMPLES; i++) {
        uint16_t km = magBuf[i], kp = phsBuf[i];
        int8_t j = i - 1;
        while (j >= 0 && magBuf[j] > km) { magBuf[j + 1] = magBuf[j]; j--; }
        magBuf[j + 1] = km;
        j = i - 1;
        while (j >= 0 && phsBuf[j] > kp) { phsBuf[j + 1] = phsBuf[j]; j--; }
        phsBuf[j + 1] = kp;
    }

    uint32_t magAcc = 0, phsAcc = 0;
    for (uint8_t i = TRIM; i < N_SAMPLES - TRIM; i++) {
        magAcc += magBuf[i];
        phsAcc += phsBuf[i];
    }

    const float n = (float)(N_SAMPLES - 2 * TRIM);
    *vmag_mV = adcToMv(magAcc / n);
    *vphs_mV = adcToMv(phsAcc / n);

    *gain_dB   = (*vmag_mV - MAG_CENTER_MV)  / MAG_SLOPE_MV_PER_DB;
    *phase_deg = (PHS_AT_0DEG_MV - *vphs_mV) / PHS_SLOPE_MV_PER_DEG;
}

void gainphase_to_gamma(float gain_dB, float phase_deg, float *gamma_mag, float *gamma_phase_deg)
{
    *gamma_mag       = powf(10.0f, gain_dB / 20.0f);
    *gamma_phase_deg = phase_deg;
}
