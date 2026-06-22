  AD8302 Reflectometer — README

  Hardware
  - Arduino UNO R4 Minima (Renesas RA4M1, 32 KB RAM, 256 KB flash)
  - AD8302 eval board
  - VMAG → A0, VPHS → A1
  - Eval board AREF pin → Arduino AREF pin (~2.4 V)
  - RFSoC UART → D0 (RX) / D1 (TX)  [Serial1, 115200 8N1]

  ADC Reference
  - Uses AR_EXTERNAL (RA4M1 core constant) — the eval board's stable ~2.4 V on the AREF pin
  - Measure actual AREF with a DMM; update AREF_MV in config.h to zero gain offset
  - The old ATmega bandgap trick does not apply to the RA4M1

  AD8302 Transfer Function
  - VMAG: gain_dB = (vmag_mV − 900) / 29    slope 29 mV/dB, 0 dB = 900 mV, range ±30 dB
  - VPHS: phase_deg = (1800 − vphs_mV) / 10  0° → 1800 mV, 180° → ~30 mV
  - Phase detector dead zones near 0° and 180° — best performance near 90°
  - Phase output is unsigned — cannot distinguish leading from lagging

  Frequency Plan
  - OSL calibration:  40–600 MHz, 5 MHz steps  → 113 points  (fixed, flight-invariant)
  - Gamma sweep:      40–600 MHz, 1 MHz steps  → 561 points  (driven by RFSoC)

  Architecture (RFSoC = master, Arduino = slave)
  - RFSoC drives all sequencing via ASCII commands over Serial1
  - Arduino responds ACK/NACK to each command/frequency step
  - After a complete 561-point sweep Arduino streams the full corrected dataset to RFSoC

  UART Protocol (115200 8N1, newline-terminated ASCII)
    RFSoC → Arduino commands:
      OSL_BEGIN       initiate OSL calibration; Arduino drives SHORT→OPEN→LOAD autonomously
      FREQ <hz>       RFSoC has tuned to this frequency; Arduino samples and ACKs
      GAMMA_BEGIN     load error terms; route SP6T to antenna, SPDT to delay; prepare sweep
      DUMP_CAL        dump error_terms[] to USB serial (bench debug only)

    Arduino → RFSoC responses:
      ACK             ready / step complete
      NACK            out-of-sequence or index overflow
      CAL_DONE        OSL complete; error terms saved to EEPROM
      DATA_BEGIN      precedes gamma dataset
      <hz>,<mag>,<ph> one OSL-corrected GammaPoint (561 lines)
      DATA_END        gamma dataset complete

  OSL Calibration Flow
  1. RFSoC sends OSL_BEGIN; Arduino ACKs, routes SP6T to SHORT
  2. RFSoC sends 339 × FREQ commands (113 per standard, same array repeated 3×)
     - After FREQ 113: Arduino autonomously switches SP6T to OPEN
     - After FREQ 226: Arduino autonomously switches SP6T to LOAD
     - After FREQ 339: Arduino computes error terms, saves to EEPROM, sends CAL_DONE

  Gamma Characterization Flow
  1. RFSoC sends GAMMA_BEGIN; Arduino loads error terms, routes SP6T to antenna,
     sets SPDT to delay, ACKs
  2. RFSoC sends 561 × FREQ commands; Arduino stores raw delay measurements, ACKs each
     - After FREQ 561: Arduino autonomously switches SPDT to thru
  3. RFSoC sends 561 × FREQ commands; Arduino stores raw thru measurements, ACKs each
     - After FREQ 561: Arduino resolves phase ambiguity, applies OSL correction,
       sends DATA_BEGIN … 561 CSV lines … DATA_END

  Phase Ambiguity Resolution
  - AD8302 phase output is unsigned (0–180°); true phase may be +θ or -θ
  - SPDT delay line introduces a known delay of 0.431 deg/MHz
  - Expected phase difference at frequency f: Δφ_expected = 0.431 × f_MHz degrees
  - If (phase_delay - phase_thru) matches +Δφ_expected → phase is positive (no flip)
  - If the difference goes the wrong direction → phase was folded; negate to correct
  - Note: Δφ_expected exceeds 180° above ~418 MHz; comparison must be done mod 360°

  File Structure
    config.h       — all #defines, structs, shared extern declarations
    adc.h/cpp      — ADC init, trimmed-mean sample(), gainphase_to_gamma()
    osl.h/cpp      — OSL cal buffer, error-term computation, EEPROM save/load, correction
    comms.h/cpp    — UART line reader, command parser, ACK/NACK/data senders
    switch.h/cpp   — SP6T and SPDT switch stubs (TODO: implement GPIO control)
    USGS_AD8302.ino — main sketch: setup/loop, state machine

  Switching
  - SPDT (switch1_thru / switch1_delay): phase ambiguity resolution
      IC: ZMSW-1211 (connectorized)
      pin2=HIGH, pin3=LOW → thru;  pin2=LOW, pin3=HIGH → delay

  - SP6T (switch2_*): calibration standard / antenna selection
      IC: JSW6-33DR+ (connectorized)
      C1(pin4) C2(pin5) C3(pin6) → port
      L        L        L        → RF1: antenna
      L        L        H        → RF2: open
      L        H        L        → RF3: short
      L        H        H        → RF4: load
      H        L        L        → RF5: terminated
      H        L        H        → RF6: terminated

  EEPROM Layout (R4 Minima: 8 KB emulated)
  - Addr 0: error_terms[113]  =  113 × 24 bytes  =  2 712 bytes

  OSL Error Terms
  The three-term error model derives one complex error term per calibration frequency
  from three known-standard measurements (Open, Short, Load).

  - Directivity (e00)
      Energy from the incident wave that leaks directly into the reflection measurement
      path without ever reaching the DUT. Even a perfectly matched DUT would produce a
      non-zero reflected reading due to finite directivity in the physical coupler.
      Numerically: e00 = GM_load (the measured reflection of the 50-ohm load).

  - Source Match (e11)
      The impedance mismatch looking back from the DUT into the measurement device.
      When the DUT reflects, part of that returning wave re-reflects off the coupler
      back toward the DUT, creating a cascading reflection effect. This is not an
      abstract term — it is a real impedance looking back from the DUT port.

  - Tracking (delta_e, also written e10*e01)
      The total transmission response of the path the reflected wave travels: from the source,
      forward through the coupler to the DUT, back from the DUT to the receiver.
      Captures all cable losses, connector interface losses, and coupler insertion loss
      in both directions. Does not include the DUT reflection itself.

  Known Calibration Notes
  - SHORT negation: AD8302 phase is unsigned; osl_store_sample() negates SHORT phase
    to place it in the lower complex half-plane for correct OSL algebra
  - Floating VMAG reads ~52 dB — verify pin connection before use
