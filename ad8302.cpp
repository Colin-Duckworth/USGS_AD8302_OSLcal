#include "ad8302.h"
#include <Arduino.h>
#include <avr/io.h>
// #include <Complex.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <EEPROM.h>

float adcVrefMv = 5000.0f;
GammaEntry osl_cal[3][num_samples];
ErrorTerms error_terms[num_samples];
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── Measure AREF using ATmega328P internal 1.1V bandgap ──────────────────────
float measureAref(void)
{
    // Switch ADC mux to internal bandgap, reference = AREF
    // ATmega2560 (Mega): bandgap = MUX[4:0]=11110, MUX5=0 in ADCSRB
    // ATmega328P  (Uno):  bandgap = MUX[3:0]=1110
#if defined(__AVR_ATmega2560__)
    ADCSRB &= ~_BV(MUX5);
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
    delay(20);   // let mux and reference settle

    uint32_t acc = 0;
    for (uint8_t i = 0; i < VREF_SAMPLES; i++) {
        ADCSRA |= _BV(ADSC);
        while (bit_is_set(ADCSRA, ADSC));
        acc += ADC;
    }

    float raw = acc / (float)VREF_SAMPLES;
    // AREF = bandgap * 1024 / raw
    float aref = (BANDGAP_MV * ADC_RESOLUTION) / raw;

    // Restore normal ADC mux (DEFAULT ref, A0)
    analogReference(DEFAULT);
    analogRead(VMAG_PIN);  // dummy read to flush mux change

    return aref;
}

// ── Helpers ───────────────────────────────────────────────────────────────────
inline float adcToMv(int counts)
{
    return counts * (adcVrefMv / ADC_RESOLUTION);
}

// ── Sample both channels, return results by pointer ──────────────────────────
void sample(float *gain_dB, float *phase_deg, float *vmag_mV, float *vphs_mV)
{
    uint16_t magAcc = 0, phsAcc = 0;

    analogRead(VMAG_PIN);   // settle mux on first channel
    for (uint8_t i = 0; i < ADC_OVERSAMPLE; i++) magAcc += analogRead(VMAG_PIN);

    analogRead(VPHS_PIN);   // settle mux on second channel
    for (uint8_t i = 0; i < ADC_OVERSAMPLE; i++) phsAcc += analogRead(VPHS_PIN);

    *vmag_mV = adcToMv(magAcc / (float)ADC_OVERSAMPLE);
    *vphs_mV = adcToMv(phsAcc / (float)ADC_OVERSAMPLE);

    *gain_dB   = (*vmag_mV - MAG_CENTER_MV)  / MAG_SLOPE_MV_PER_DB;
    *phase_deg = (PHS_AT_0DEG_MV - *vphs_mV) / PHS_SLOPE_MV_PER_DEG;
}


// ── Mode 0: compact CSV for host software ────────────────────────────────────
void sampleAndSend(void)
{
    float gain_dB, phase_deg, vmag_mV, vphs_mV;
    sample(&gain_dB, &phase_deg, &vmag_mV, &vphs_mV);

    Serial.print(gain_dB,   2);
    Serial.print(',');
    Serial.print(phase_deg, 2);
    Serial.print("\r\n");
}

// ── Mode 1: human-readable poll for Serial Monitor + OLED ────────────────────
void sampleAndPrint(void)
{
    float gain_dB, phase_deg, vmag_mV, vphs_mV;
    sample(&gain_dB, &phase_deg, &vmag_mV, &vphs_mV);

    // Serial monitor print
    Serial.print(F("Gain: "));
    Serial.print(gain_dB,   2);
    Serial.print(F(" dB  (VMAG: "));
    Serial.print(vmag_mV,   1);
    Serial.print(F(" mV)    Phase: "));
    Serial.print(phase_deg, 2);
    Serial.print(F(" deg  (VPHS: "));
    Serial.print(vphs_mV,   1);
    Serial.print(F(" mV)    AREF: "));
    Serial.print(adcVrefMv, 1);
    Serial.println(F(" mV"));

    // OLED print to display                                                                                                                                                  
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.print(F("Gain: "));
    display.print(gain_dB, 2);
    display.print(F(" dB"));

    display.setCursor(0, 20);
    display.print(F("Phase: "));
    display.print(phase_deg, 2);
    display.print(F(" deg"));

    display.setCursor(0, 40);
    display.print(F("AREF: "));
    display.print(adcVrefMv, 1);
    display.print(F(" mV"));

    display.display();
}

// startup print for serial monitor mode (no UART to RFsoc mode)
void setup_print_mode1(void)
{
    Serial.println(F("AD8302 — Serial Monitor mode"));
    Serial.print  (F("Polling every "));
    Serial.print  (POLL_INTERVAL_MS);
    Serial.println(F(" ms"));
    Serial.print  (F("Initial AREF: "));
    Serial.print  (adcVrefMv, 1);
    Serial.println(F(" mV"));
    Serial.println(F("-----------------------------"));
}

// Converts gain/loss properties to gamma properties
void gainphase_to_gamma(float gain_dB, float phase_degrees, float *gamma_mag, float *gamma_phase_deg)
{
    // convert gain in dB to linear magnitude
    *gamma_mag = pow(10.0f, gain_dB / 20.0f);

    // phase is already in degrees, just pass it through
    *gamma_phase_deg = phase_degrees;
}

bool buttonPressed(void)
{
    if (digitalRead(BTN_PIN) == LOW)
    {
        delay(200);  // debounce
        if (digitalRead(BTN_PIN) == LOW){ // if still low after 200ms
            return true;    
        }
    }
    return false;
}

void OSL_calibration_initiate()
{
    const char* standardLabels[] = {"OPEN", "SHORT", "LOAD"};

    for (int i = 0; i < 3; i++) // 0 = open, 1 = short, 2 = 50-ohm load
    {
        for (int j = 0; j < num_samples; j++)
        {
            float gain_dB, phase_deg, vmag_mV, vphs_mV;

            Serial.print(F("DBG: OSL=")); Serial.print(i); Serial.print(F(" freq=")); Serial.println(j);
            // live update loop: refresh readings on OLED until button pressed

            do {
                sample(&gain_dB, &phase_deg, &vmag_mV, &vphs_mV);

                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);

                display.setCursor(0, 0);
                display.print(standardLabels[i]);
                display.print(F(" "));
                display.print((float)freq_points[j] / 1.0e6f, 0);
                display.print(F(" MHz"));

                display.setCursor(0, 20);
                display.print(F("Gain: "));
                display.print(gain_dB, 2);
                display.print(F(" dB"));

                display.setCursor(0, 40);
                display.print(F("Phase: "));
                display.print(phase_deg, 2);
                display.print(F(" deg"));

                display.display();
                delay(200);

            } while (!buttonPressed());

            float gamma_mag, gamma_phase_deg;
            gainphase_to_gamma(gain_dB, phase_deg, &gamma_mag, &gamma_phase_deg);

            osl_cal[i][j].frequency_hz    = (float)freq_points[j];
            osl_cal[i][j].gamma_mag       = gamma_mag;
            osl_cal[i][j].gamma_phase_deg = gamma_phase_deg;

            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.print(standardLabels[i]);
            display.print(F(" "));
            display.print((float)freq_points[j] / 1.0e6f, 0);
            display.print(F(" MHz"));
            display.setCursor(0, 28);
            display.print(F("Entry saved."));
            display.display();
            delay(500);
        }
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("OSL done."));
    display.println(F("Computing error"));
    display.println(F("terms..."));
    display.display();
}

void compute_error_terms(void)
{
    for (int j = 0; j < num_samples; j++)
    {
        // convert each standard's polar measurement to rectangular complex
        float o_rad = osl_cal[0][j].gamma_phase_deg * PI / 180.0f;
        float o_re  = osl_cal[0][j].gamma_mag * cos(o_rad);
        float o_im  = osl_cal[0][j].gamma_mag * sin(o_rad);

        float s_rad = -osl_cal[1][j].gamma_phase_deg * PI / 180.0f;  // negated: AD8302 always positive, short is lower half-plane. (check datasheet is phase ambigous must correct)
        float s_re  = osl_cal[1][j].gamma_mag * cos(s_rad);
        float s_im  = osl_cal[1][j].gamma_mag * sin(s_rad);

        float l_rad = osl_cal[2][j].gamma_phase_deg * PI / 180.0f;
        float l_re  = osl_cal[2][j].gamma_mag * cos(l_rad);
        float l_im  = osl_cal[2][j].gamma_mag * sin(l_rad);

        // e00 = ΓM_load
        error_terms[j].e00_re = l_re;
        error_terms[j].e00_im = l_im;

        // e11 = (ΓM_open + ΓM_short - 2*ΓM_load) / (ΓM_open - ΓM_short)
        float num_re = o_re + s_re - 2.0f * l_re;
        float num_im = o_im + s_im - 2.0f * l_im;
        float den_re = o_re - s_re;
        float den_im = o_im - s_im;
        float den_sq = den_re*den_re + den_im*den_im;

        error_terms[j].e11_re = (num_re*den_re + num_im*den_im) / den_sq;
        error_terms[j].e11_im = (num_im*den_re - num_re*den_im) / den_sq;

        // delta_e = ΓM_open*e11 - (ΓM_open - ΓM_load)
        float prod_re = o_re*error_terms[j].e11_re - o_im*error_terms[j].e11_im;
        float prod_im = o_re*error_terms[j].e11_im + o_im*error_terms[j].e11_re;

        error_terms[j].delta_e_re = prod_re - o_re + l_re;
        error_terms[j].delta_e_im = prod_im - o_im + l_im;
    }

    EEPROM.put(CAL_EEPROM_ADDR, error_terms);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Cal complete!"));
    display.println(F("Saved to EEPROM."));
    display.display();
}

double freq_points[num_samples];

void generate_freq_points(void)
{
    double step = (f_max - f_min) / (num_samples - 1);

    for (int i = 0; i < num_samples; i++)
    {
        freq_points[i] = f_min + i * step;
    }
}

void OLED_startup(void)
{
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Booting up..."));
    display.display();
    delay(1000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("AD8302 Sampler"));
    display.println(F("Initializing..."));
    display.display();
}




