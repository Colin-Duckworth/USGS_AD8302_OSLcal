#pragma once
#include <Arduino.h>
#include <EEPROM.h>

// ── Frequency plan ────────────────────────────────────────────────────────────
// OSL calibration: fixed 5 MHz grid, flight-invariant
#define F_CAL_MIN_HZ      40000000.0
#define F_CAL_MAX_HZ     600000000.0
#define F_CAL_STEP_HZ      5000000.0
#define N_CAL_POINTS             113   // (600-40)/5 + 1

// Gamma sweep: 1 MHz grid, driven by RFSoC FREQ commands
#define F_SWEEP_MIN_HZ    40000000.0
#define F_SWEEP_MAX_HZ   600000000.0
#define F_SWEEP_STEP_HZ    1000000.0
#define N_SWEEP_POINTS           561   // (600-40)/1 + 1

// ── Pin assignments ───────────────────────────────────────────────────────────
#define VMAG_PIN  A0
#define VPHS_PIN  A1

// SPDT (switch1): phase ambiguity resolution
// pin2=HIGH, pin3=LOW → thru (ON);  pin2=LOW, pin3=HIGH → delay (OFF)
#define SPDT_A_PIN  2
#define SPDT_B_PIN  3

// SP6T (switch2): calibration standard / antenna selection
// Control lines: pin4=C1, pin5=C2, pin6=C3
// C1 C2 C3 → port
// L  L  L  → RF1 (antenna)
// L  L  H  → RF2 (open)
// L  H  L  → RF3 (short)
// L  H  H  → RF4 (load)
// H  L  L  → RF5 (terminated)
// H  L  H  → RF6 (terminated)
#define SP6T_C1_PIN  4
#define SP6T_C2_PIN  5
#define SP6T_C3_PIN  6

// ── ADC configuration ─────────────────────────────────────────────────────────
// R4 Minima uses AR_EXTERNAL — the AD8302 eval board provides stable ~2.4 V on AREF.
// Measure actual AREF with a DMM and update AREF_MV to zero any gain offset.
#define AREF_MV         2400.0f
#define ADC_RESOLUTION  1024.0f   // 10-bit Arduino default mode
#define N_SAMPLES            8    // readings taken per channel per measurement
#define TRIM                 2    // discard lowest 2 and highest 2; average middle 4

// ── Settling delays ───────────────────────────────────────────────────────────
// All values are 2× the worst-case datasheet spec for margin.
// SP6T (JSW6-33DR+):  datasheet 2 µs → 4 µs applied after any switch2_*() call
// SPDT (ZMSW-1211):   datasheet 5 µs → 10 µs applied after any switch1_*() call
// AD8302:             VPHS 120° settle to 1% = 500 ns → 1 µs applied before each sample
#define DELAY_SP6T_US    4
#define DELAY_SPDT_US   10
#define DELAY_AD8302_US  1

// ── Phase ambiguity resolution ────────────────────────────────────────────────
// Characterised with VNA. Update if delay line is changed.
#define DELAY_DEG_PER_MHZ  0.25f   // degrees of extra phase added by SPDT delay path per MHz

// ── AD8302 transfer-function constants ────────────────────────────────────────
#define MAG_SLOPE_MV_PER_DB   29.0f
#define MAG_CENTER_MV        900.0f
#define PHS_SLOPE_MV_PER_DEG  10.0f
#define PHS_AT_0DEG_MV      1800.0f

// ── UART ──────────────────────────────────────────────────────────────────────
#define UART_BAUD     115200
#define RFSOC_SERIAL  Serial1   // hardware UART on D0 (RX) / D1 (TX)
#define DEBUG_SERIAL  Serial    // USB-CDC for bench debug only

// ── EEPROM ────────────────────────────────────────────────────────────────────
// error_terms[N_CAL_POINTS] occupies (113 * 24) = 2712 bytes starting at addr 0.
// R4 Minima EEPROM emulation provides 8 KB — well within budget.
#define CAL_EEPROM_ADDR  0

// ── Data structures ───────────────────────────────────────────────────────────
struct GammaPoint {
    float gamma_mag;
    float gamma_phase_deg;
    // frequency reconstructed from index: F_SWEEP_MIN_HZ + i * F_SWEEP_STEP_HZ
};

struct ErrorTerms {
    float e00_re,     e00_im;
    float e11_re,     e11_im;
    float delta_e_re, delta_e_im;
};

struct RawSample {
    float gain_dB;
    float phase_deg;
};

// ── is_mirrored bit array ─────────────────────────────────────────────────────
// One bit per sweep point. ceil(561/8) = 71 bytes.
#define IS_MIRRORED_BYTES  ((N_SWEEP_POINTS + 7) / 8)
#define IS_MIRRORED_SET(arr, i)  ((arr)[(i)/8] |=  (1u << ((i)%8)))
#define IS_MIRRORED_GET(arr, i)  (((arr)[(i)/8] >> ((i)%8)) & 1u)

// ── Shared globals (defined in osl.cpp) ───────────────────────────────────────
extern ErrorTerms error_terms[N_CAL_POINTS];
extern GammaPoint sweep_results[N_SWEEP_POINTS];
extern RawSample  gamma_delay_raw[N_SWEEP_POINTS];
extern RawSample  gamma_thru_raw[N_SWEEP_POINTS];
extern uint8_t    is_mirrored[IS_MIRRORED_BYTES];
