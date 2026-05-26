// Declares functions to run script

#pragma once

// ── Select Mode ───────────────────────────────────────────────────────────────
// 0 = UART command mode : send 'S' (0x53), receive "gain_dB,phase_deg\r\n"
// 1 = Serial monitor mode: polls every POLL_INTERVAL_MS, prints human-readable
// 2 = OSL calibration mode: Loops through frequency band and takes OSL readings
// 3 = Prints error terms from EEPROM into the serial display for data analysis (under development)

#define MODE 2
#define POLL_INTERVAL_MS    2000    // (mode 1 only) ms between samples
#define VREF_UPDATE_MS     10000    // how often to re-measure AREF (ms)
#define VREF_SAMPLES           8    // number of bandgap reads to average
#define ADC_OVERSAMPLE        10    // readings per channel averaged in sample()
#define f_min               50e6    // minimum frequency 50MHz
#define f_max              600e6    // maximum frequency 600MHz
#define num_samples           12    // frequency points for OSL calibration

// ── Pin assignment ─────────────────────────────────────────────────────────────
#define VMAG_PIN  A0
#define VPHS_PIN  A1
#define BTN_PIN    6

// ── ADC reference ─────────────────────────────────────────────────────────────
#define ADC_RESOLUTION 1024.0f
#define BANDGAP_MV     1100.0f

// ── AD8302 transfer-function constants ────────────────────────────────────────
#define MAG_SLOPE_MV_PER_DB   29.0f
#define MAG_CENTER_MV        900.0f
#define PHS_SLOPE_MV_PER_DEG  10.0f
#define PHS_AT_0DEG_MV      1800.0f

// OLED Display Config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// EEPROM
#define CAL_EEPROM_ADDR 0

// Sample definitions
#define N_SAMPLES 16
#define TRIM      4   // discard bottom 4 and top 4 (middle 8 averaged)

struct GammaEntry { //defines data structure that includes gamma per frequency
    float frequency_hz;
    float gamma_mag;
    float gamma_phase_deg;
};

struct ErrorTerms {  // complex error terms at one frequency point
    float e00_re,     e00_im;
    float e11_re,     e11_im;
    float delta_e_re, delta_e_im;
};

extern GammaEntry osl_cal[3][num_samples];
extern ErrorTerms error_terms[num_samples];

extern float adcVrefMv;

float measureAref(void);
void sample(float *gain_dB, float *phase_deg, float *vmag_mV, float *vphs_mV);
void sampleAndSend(void);
void sampleAndPrint(void);
void setup_print_mode1(void);
void gainphase_to_gamma(float gain_dB, float phase_degrees, float *gamma_mag, float *gamma_phase_deg);
bool buttonPressed(void);
void OSL_calibration_initiate(void);
void compute_error_terms(void);
void OLED_startup(void);

extern double freq_points[num_samples];
void generate_freq_points(void);
