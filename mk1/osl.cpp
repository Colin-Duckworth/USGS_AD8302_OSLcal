#include "osl.h"
#include "adc.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// ── Shared globals ────────────────────────────────────────────────────────────
ErrorTerms error_terms[N_CAL_POINTS];
GammaPoint sweep_results[N_SWEEP_POINTS];

// Transient buffer — needed only during calibration, never persisted.
// 3 standards × 113 points × 12 bytes = 4 068 bytes.
static GammaPoint osl_cal[3][N_CAL_POINTS];

// ── Nearest calibration grid index (uniform grid, O(1)) ──────────────────────
static int calIndex(double freq_hz)
{
    int idx = (int)((freq_hz - F_CAL_MIN_HZ) / F_CAL_STEP_HZ + 0.5);
    if (idx < 0)              idx = 0;
    if (idx >= N_CAL_POINTS)  idx = N_CAL_POINTS - 1;
    return idx;
}

// ── Public API ────────────────────────────────────────────────────────────────

void osl_store_sample(int std_idx, int freq_idx, double freq_hz)
{
    float gain_dB, phase_deg, vmag_mV, vphs_mV;
    sample(&gain_dB, &phase_deg, &vmag_mV, &vphs_mV);

    float gamma_mag, gamma_phase;
    gainphase_to_gamma(gain_dB, phase_deg, &gamma_mag, &gamma_phase);

    // AD8302 phase output is unsigned (0–180°). SHORT ideally reflects at 180°,
    // which in the lower complex half-plane means a negative phase. Negate here
    // so the OSL algebra places it correctly in rectangular coordinates.
    if (std_idx == 1) gamma_phase = -gamma_phase;

    osl_cal[std_idx][freq_idx].frequency_hz    = (float)freq_hz;
    osl_cal[std_idx][freq_idx].gamma_mag       = gamma_mag;
    osl_cal[std_idx][freq_idx].gamma_phase_deg = gamma_phase;
}

void osl_compute_error_terms(void)
{
    for (int j = 0; j < N_CAL_POINTS; j++) {
        // Convert each standard's polar measurement to rectangular complex
        float o_rad = osl_cal[0][j].gamma_phase_deg * (PI / 180.0f);
        float o_re  = osl_cal[0][j].gamma_mag * cosf(o_rad);
        float o_im  = osl_cal[0][j].gamma_mag * sinf(o_rad);

        float s_rad = osl_cal[1][j].gamma_phase_deg * (PI / 180.0f);
        float s_re  = osl_cal[1][j].gamma_mag * cosf(s_rad);
        float s_im  = osl_cal[1][j].gamma_mag * sinf(s_rad);

        float l_rad = osl_cal[2][j].gamma_phase_deg * (PI / 180.0f);
        float l_re  = osl_cal[2][j].gamma_mag * cosf(l_rad);
        float l_im  = osl_cal[2][j].gamma_mag * sinf(l_rad);

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

GammaPoint osl_correct(double freq_hz, float gain_dB, float phase_deg)
{
    float raw_mag, raw_phase;
    gainphase_to_gamma(gain_dB, phase_deg, &raw_mag, &raw_phase);

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

    GammaPoint result;
    result.frequency_hz    = (float)freq_hz;
    result.gamma_mag       = sqrtf(a_re * a_re + a_im * a_im);
    result.gamma_phase_deg = atan2f(a_im, a_re) * (180.0f / PI);
    return result;
}

void osl_dump(void)
{
    DEBUG_SERIAL.println(F("=== ERROR TERMS ==="));
    DEBUG_SERIAL.println(F("freq_hz,e00_re,e00_im,e11_re,e11_im,delta_e_re,delta_e_im"));
    for (int j = 0; j < N_CAL_POINTS; j++) {
        float freq = F_CAL_MIN_HZ + j * F_CAL_STEP_HZ;
        DEBUG_SERIAL.print((long)freq);                      DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].e00_re,     6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].e00_im,     6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].e11_re,     6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].e11_im,     6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.print(error_terms[j].delta_e_re, 6);   DEBUG_SERIAL.print(',');
        DEBUG_SERIAL.println(error_terms[j].delta_e_im, 6);
    }
    DEBUG_SERIAL.println(F("--- dump complete ---"));
}
