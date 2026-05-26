  AD8302 Reflectometer Sampler — README

  Hardware
  - Arduino Uno (ATmega328P)
  - AD8302 eval board
  - VMAG → A0, VPHS → A1
  - Eval board pulls AREF to ~2.4V (not 5V) — AREF is measured at runtime via ATmega328P internal 1.1V bandgap, do not hardcode

  AD8302 Transfer Function
  - VMAG: gain_dB = (vmag_mV − 900) / 29 — slope 29 mV/dB, 0 dB = 900 mV, range ±30 dB
  - VPHS: phase_deg = (1800 − vphs_mV) / 10 — 0° → 1800 mV, 180° → ~30 mV
  - Phase detector dead zones near 0° and 180° — best performance near 90°
  - Phase output is unsigned — cannot distinguish leading from lagging

  Operating Modes
  - Mode 0: UART command mode — send S, receive gain_dB,phase_deg\r\n (115200 baud 8N1)
  - Mode 1: Serial monitor auto-poll, human-readable, every 2s
  - Select via #define MODE in ad8302.h

  Known Bring-Up Issues
  - Floating VMAG pin reads ~52 dB — verify pin connection first
  - With inputs shorted: ~1.58 dB offset observed, adjust MAG_CENTER_MV to 947.4 mV to zero it

  Calibration (in development)
  - 1-port 3-term error model (OSL) — open/short/load standards
  - Corrects for directivity (e00), port match (e11), and tracking errors (e10*e01)
  *Note*
  this is distinct from the startup gamma(f) characterzation, OSL makes it so we can measure accurate gammas, gamma(f) characteriztaion lets us actually get those gammas


  Functionality to be implemented
  OSL procedure
  functionality to get stable readings at some frequency (maybe by taking say 20 samples and removing the top and bottom 5, averaging the middle 10)

  OLED Display example
  // Update display
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Freq: ");
  display.print(baseFreq);
  display.print(" ");
  display.println(unitLabel);

  display.setCursor(0, 20);
  display.print("Target: ");
  display.println(targetFreq);

  display.setCursor(0, 40);
  display.print("Set: ");
  display.println(lastFreq);

  display.display();
  
