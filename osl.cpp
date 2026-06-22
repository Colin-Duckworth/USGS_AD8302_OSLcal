#include "osl.h"
#include "adc.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// ── Shared globals ────────────────────────────────────────────────────────────
ErrorTerms error_terms[N_CAL_POINTS];
GammaPoint sweep_results[N_SWEEP_POINTS];
RawSample  gamma_delay_raw[N_SWEEP_POINTS];
RawSample  gamma_thru_raw[N_SWEEP_POINTS];
uint8_t    is_mirrored[IS_MIRRORED_BYTES];

// Transient OSL buffer — needed only during calibration, never persisted.
// std_idx: 0=SHORT, 1=OPEN, 2=LOAD
// Stored as int16_t complex scaled ×10000 (gamma re/im range ±1.0 → ±10000).
// 2 × 3 × 113 × 2 bytes = 1 356 bytes (vs 2 712 as floats).
static int16_t osl_re[3][N_CAL_POINTS];
static int16_t osl_im[3][N_CAL_POINTS];

// ── Nearest calibration grid index (uniform grid, O(1)) ──────────────────────
static int calIndex(double freq_hz)
{
    int idx = (int)((freq_hz - F_CAL_MIN_HZ) / F_CAL_STEP_HZ + 0.5);
    if (idx < 0)             idx = 0;
    if (idx >= N_CAL_POINTS) idx = N_CAL_POINTS - 1;
    return idx;
}

// ── Public API ────────────────────────────────────────────────────────────────

void osl_store_sample(int std_idx, int freq_idx, double freq_hz)
{
    delayMicroseconds(DELAY_AD8302_US);
    float gain_dB, phase_deg, vmag_mV, vphs_mV;
    sample(&gain_dB, &phase_deg, &vmag_mV, &vphs_mV);

    float gamma_mag, gamma_phase;
    gainphase_to_gamma(gain_dB, phase_deg, &gamma_mag, &gamma_phase);

    // std_idx 0 = SHORT: negate phase to place SHORT in the lower
    // complex half-plane for correct OSL algebra.
    float rad = gamma_phase * (PI / 180.0f);
    float re  =  gamma_mag * cosf(rad);
    float im  =  gamma_mag * sinf(rad);
    if (std_idx == 0) im = -im;   // phase negation → im negation

    osl_re[std_idx][freq_idx] = (int16_t)(re * 10000.0f);
    osl_im[std_idx][freq_idx] = (int16_t)(im * 10000.0f);
}

void osl_compute_error_terms(void)
{
    // osl_cal indices: 0=SHORT, 1=OPEN, 2=LOAD
    for (int j = 0; j < N_CAL_POINTS; j++) {
        float o_re = osl_re[1][j] * 1.0e-4f;
        float o_im = osl_im[1][j] * 1.0e-4f;

        float s_re = osl_re[0][j] * 1.0e-4f;
        float s_im = osl_im[0][j] * 1.0e-4f;

        float l_re = osl_re[2][j] * 1.0e-4f;
        float l_im = osl_im[2][j] * 1.0e-4f;

        // e00 = ΓM_load
        error_terms[j].e00_re = l_re;
        error_terms[j].e00_im = l_im;

        // e11 = (ΓM_open + ΓM_short − 2·ΓM_load) / (ΓM_open − ΓM_short)
        float num_re = o_re + s_re - 2.0f * l_re;
        float num_im = o_im + s_im - 2.0f * l_im;
        float den_re = o_re - s_re;
        float den_im = o_im - s_im;
        float den_sq = den_re*den_re + den_im*den_im;

        error_terms[j].e11_re = (num_re*den_re + num_im*den_im) / den_sq;
        error_terms[j].e11_im = (num_im*den_re - num_re*den_im) / den_sq;

        // delta_e = (ΓM_open − ΓM_load) · (1 − e11)
        float A_re = o_re - l_re;
        float A_im = o_im - l_im;
        float B_re = 1.0f - error_terms[j].e11_re;
        float B_im =       -error_terms[j].e11_im;

        error_terms[j].delta_e_re = A_re*B_re - A_im*B_im;
        error_terms[j].delta_e_im = A_re*B_im + A_im*B_re;
    }

    EEPROM.put(CAL_EEPROM_ADDR, error_terms);
    DEBUG_SERIAL.println(F("OSL complete. Error terms saved to EEPROM."));
}

void osl_load_from_eeprom(void)
{
    EEPROM.get(CAL_EEPROM_ADDR, error_terms);
}

void osl_apply_corrections(void)
{
    // ── Pass 1: build is_mirrored bit array ───────────────────────────────────
    memset(is_mirrored, 0, IS_MIRRORED_BYTES);

    for (int i = 0; i < N_SWEEP_POINTS; i++) {
        float f_mhz     = (float)(F_SWEEP_MIN_HZ / 1.0e6) + i * (float)(F_SWEEP_STEP_HZ / 1.0e6);
        float delta_phi = DELAY_DEG_PER_MHZ * f_mhz;

        float phi_thru  = gamma_thru_raw[i].phase_deg;
        float phi_delay = gamma_delay_raw[i].phase_deg;

        // Case 1: true phase = +phi_thru (not mirrored)
        float pred1 = fabsf(phi_thru - delta_phi);
        float err1  = fabsf(phi_delay - pred1);

        // Case 2: true phase = -phi_thru (mirrored)
        float pred2_raw = phi_thru + delta_phi;
        float pred2     = (pred2_raw <= 180.0f) ? pred2_raw : 360.0f - pred2_raw;
        float err2      = fabsf(phi_delay - pred2);

        if (err2 < err1) IS_MIRRORED_SET(is_mirrored, i);
    }

    // ── Pass 2: resolve phase, apply OSL correction ───────────────────────────
    for (int i = 0; i < N_SWEEP_POINTS; i++) {
        double freq_hz  = F_SWEEP_MIN_HZ + i * F_SWEEP_STEP_HZ;

        float true_phase = IS_MIRRORED_GET(is_mirrored, i)
                           ? -gamma_thru_raw[i].phase_deg
                           :  gamma_thru_raw[i].phase_deg;

        float raw_mag, raw_phase;
        gainphase_to_gamma(gamma_thru_raw[i].gain_dB, true_phase, &raw_mag, &raw_phase);

        float m_rad = raw_phase * (PI / 180.0f);
        float m_re  = raw_mag * cosf(m_rad);
        float m_im  = raw_mag * sinf(m_rad);

        int j = calIndex(freq_hz);

        float e00_re = error_terms[j].e00_re, e00_im = error_terms[j].e00_im;
        float e11_re = error_terms[j].e11_re, e11_im = error_terms[j].e11_im;
        float de_re  = error_terms[j].delta_e_re, de_im = error_terms[j].delta_e_im;

        // ΓA = (ΓM − e00) / (Δe + e11·(ΓM − e00))
        float num_re = m_re - e00_re;
        float num_im = m_im - e00_im;

        float e11n_re = e11_re * num_re - e11_im * num_im;
        float e11n_im = e11_re * num_im + e11_im * num_re;

        float den_re = de_re + e11n_re;
        float den_im = de_im + e11n_im;
        float den_sq = den_re * den_re + den_im * den_im;

        float a_re = (num_re * den_re + num_im * den_im) / den_sq;
        float a_im = (num_im * den_re - num_re * den_im) / den_sq;

        sweep_results[i].gamma_mag       = sqrtf(a_re * a_re + a_im * a_im);
        sweep_results[i].gamma_phase_deg = atan2f(a_im, a_re) * (180.0f / PI);
    }
}

void osl_dump(void)
{
    DEBUG_SERIAL.println(F("=== ERROR TERMS ==="));
    DEBUG_SERIAL.println(F("freq_hz,e00_re,e00_im,e11_re,e11_im,delta_e_re,delta_e_im"));
    for (int j = 0; j < N_CAL_POINTS; j++) {
        long freq = (long)(F_CAL_MIN_HZ + j * F_CAL_STEP_HZ);
        DEBUG_SERIAL.print(freq);                      DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].e00_re,     6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].e00_im,     6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].e11_re,     6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].e11_im,     6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].delta_e_re, 6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.println(error_terms[j].delta_e_im, 6);
    }
    DEBUG_SERIAL.println(F("--- dump complete ---"));
}
