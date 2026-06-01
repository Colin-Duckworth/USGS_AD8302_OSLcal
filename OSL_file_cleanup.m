%% OSL_disambig.m
clc; clear all; close all;

% -------------------------------------------------------------------------
% File paths
% -------------------------------------------------------------------------
rfm_file_OPEN  = 'AD8302_OPEN_52pts.txt';
rfm_file_SHORT = 'AD8302_SHORT_52pts.txt';
rfm_file_LOAD  = 'AD8302_LOAD_52pts.txt';

% =========================================================================
% Load all three standards
% =========================================================================
osl_open  = load_ad8302(rfm_file_OPEN);
osl_short = load_ad8302(rfm_file_SHORT);
osl_load  = load_ad8302(rfm_file_LOAD);

f_open  = osl_open.freq_hz  / 1e6;   % MHz
f_short = osl_short.freq_hz / 1e6;
f_load  = osl_load.freq_hz  / 1e6;

mag_open  = 20*log10(abs(osl_open.gamma));
mag_short = 20*log10(abs(osl_short.gamma));
mag_load  = 20*log10(abs(osl_load.gamma));

ph_open  = rad2deg(angle(osl_open.gamma));
ph_short = rad2deg(angle(osl_short.gamma));
ph_load  = rad2deg(angle(osl_load.gamma));

% =========================================================================
% Ideal reflection coefficients for each calibration standard
%
%   Open  -> Gamma_ideal = +1        (|Gamma|=1, phase=0 deg)
%   Short -> Gamma_ideal = -1        (|Gamma|=1, phase=180 deg)
%   Load  -> Gamma_ideal =  0        (matched termination)
%
%   These are the textbook ideal values.  Replace with measured standard
%   data (e.g. from a cal-kit definition file) for higher accuracy.
% =========================================================================
N = numel(osl_open.freq_hz);

Gamma_open_ideal  =  ones(N,1);          % +1
Gamma_short_ideal = -ones(N,1);          % -1
Gamma_load_ideal  =  zeros(N,1);         %  0

% =========================================================================
% Solve for the 3-term error model at each frequency point
%
%   From the one-port bilinear error model (Agilent slide 14):
%
%       Gamma_M = e00 + (e10*e01)*Gamma / (1 - e11*Gamma)
%
%   Rearranged into linear form:
%
%       e00 + Gamma_i * Gamma_Mi * e11 - Gamma_i * Delta_e = Gamma_Mi
%
%   where  Delta_e = e00*e11 - (e10*e01)
%
%   For the three standards this gives a 3x3 system per frequency:
%
%       [1   G1*GM1  -G1 ] [e00    ]   [GM1]
%       [1   G2*GM2  -G2 ] [e11    ] = [GM2]
%       [1   G3*GM3  -G3 ] [Delta_e]   [GM3]
%
%   Solved via MATLAB's backslash (\) operator.
% =========================================================================
e00    = zeros(N,1);   % Directivity
e11    = zeros(N,1);   % Port match
Delta_e = zeros(N,1);  % e00*e11 - e10*e01  (tracking composite)
e10e01  = zeros(N,1);  % Reflection tracking  (recovered after solve)

GM1 = osl_open.gamma;
GM2 = osl_short.gamma;
GM3 = osl_load.gamma;

G1  = Gamma_open_ideal;
G2  = Gamma_short_ideal;
G3  = Gamma_load_ideal;

for k = 1:N
    A = [ 1,  G1(k)*GM1(k), -G1(k) ;
          1,  G2(k)*GM2(k), -G2(k) ;
          1,  G3(k)*GM3(k), -G3(k) ];

    b = [ GM1(k); GM2(k); GM3(k) ];

    x = A \ b;            % least-squares solve (exact for 3x3)

    e00(k)     = x(1);
    e11(k)     = x(2);
    Delta_e(k) = x(3);
end

% Recover reflection tracking from composite term
e10e01 = e00 .* e11 - Delta_e;

fprintf('\n--- Error term summary (first 5 frequency points) ---\n');
fprintf('%10s  %12s  %12s  %12s  %12s\n', ...
    'Freq(MHz)', '|e00|(dB)', '|e11|(dB)', '|e10e01|(dB)', '|Delta_e|(dB)');
for k = 1:N
    fprintf('%10.2f  %12.3f  %12.3f  %12.3f  %12.3f\n', ...
        osl_open.freq_hz(k)/1e6, ...
        20*log10(abs(e00(k))+eps), ...
        20*log10(abs(e11(k))+eps), ...
        20*log10(abs(e10e01(k))+eps), ...
        20*log10(abs(Delta_e(k))+eps));
end

% =========================================================================
% Apply calibration: correct a raw DUT measurement
%
%   Inverted bilinear (Agilent slide 14, "Actual"):
%
%       Gamma_DUT = (Gamma_M - e00) / (Gamma_M * e11 - Delta_e)
%
%   Replace 'rfm_file_DUT' with your actual DUT measurement file.
%   Comment out this block if no DUT file is available.
% =========================================================================
rfm_file_DUT = '';          % <-- set to your DUT file path to enable

if ~isempty(rfm_file_DUT)
    osl_dut   = load_ad8302(rfm_file_DUT);
    GM_dut    = osl_dut.gamma;

    Gamma_DUT = (GM_dut - e00) ./ (GM_dut .* e11 - Delta_e);

    mag_dut_raw  = 20*log10(abs(GM_dut));
    mag_dut_corr = 20*log10(abs(Gamma_DUT));
    ph_dut_raw   = rad2deg(angle(GM_dut));
    ph_dut_corr  = rad2deg(angle(Gamma_DUT));

    figure('Name','DUT: Raw vs Corrected S11');
    subplot(2,1,1);
    plot(f_open, mag_dut_raw, 'r--', f_open, mag_dut_corr, 'b-', 'LineWidth',1.5);
    xlabel('Frequency (MHz)'); ylabel('|S11| (dB)');
    title('DUT Reflection Magnitude'); legend('Raw','Calibrated'); grid on;

    subplot(2,1,2);
    plot(f_open, ph_dut_raw, 'r--', f_open, ph_dut_corr, 'b-', 'LineWidth',1.5);
    xlabel('Frequency (MHz)'); ylabel('Phase (deg)');
    title('DUT Reflection Phase'); legend('Raw','Calibrated'); grid on;
end

% =========================================================================
% Plot error terms vs frequency
% =========================================================================
freq_MHz = osl_open.freq_hz / 1e6;

figure('Name','1-Port Error Terms');

subplot(3,2,1);
plot(freq_MHz, 20*log10(abs(e00)+eps), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('dB');
title('e_{00} – Directivity (magnitude)'); grid on;

subplot(3,2,2);
plot(freq_MHz, rad2deg(angle(e00)), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('deg');
title('e_{00} – Directivity (phase)'); grid on;

subplot(3,2,3);
plot(freq_MHz, 20*log10(abs(e11)+eps), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('dB');
title('e_{11} – Port Match (magnitude)'); grid on;

subplot(3,2,4);
plot(freq_MHz, rad2deg(angle(e11)), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('deg');
title('e_{11} – Port Match (phase)'); grid on;

subplot(3,2,5);
plot(freq_MHz, 20*log10(abs(e10e01)+eps), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('dB');
title('e_{10}e_{01} – Tracking (magnitude)'); grid on;

subplot(3,2,6);
plot(freq_MHz, rad2deg(angle(e10e01)), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('deg');
title('e_{10}e_{01} – Tracking (phase)'); grid on;

sgtitle('One-Port 3-Term Error Model Calibration Results');

% =========================================================================
% Verify calibration: apply correction to the cal standards themselves
% (corrected values should collapse to their ideal Gamma)
% =========================================================================
Gamma_open_corr  = (GM1 - e00) ./ (GM1 .* e11 - Delta_e);
Gamma_short_corr = (GM2 - e00) ./ (GM2 .* e11 - Delta_e);
Gamma_load_corr  = (GM3 - e00) ./ (GM3 .* e11 - Delta_e);

err_open  = abs(Gamma_open_corr  - G1);
err_short = abs(Gamma_short_corr - G2);
err_load  = abs(Gamma_load_corr  - G3);

fprintf('\n--- Calibration self-check (residual |error| vs ideal) ---\n');
fprintf('  OPEN  max residual: %.2e\n', max(err_open));
fprintf('  SHORT max residual: %.2e\n', max(err_short));
fprintf('  LOAD  max residual: %.2e\n', max(err_load));

figure('Name','Cal Self-Check: Corrected Standards vs Ideal');
subplot(3,1,1);
plot(freq_MHz, 20*log10(err_open+eps), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('|Error| (dB)');
title('OPEN: |Corrected - Ideal|'); grid on;

subplot(3,1,2);
plot(freq_MHz, 20*log10(err_short+eps), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('|Error| (dB)');
title('SHORT: |Corrected - Ideal|'); grid on;

subplot(3,1,3);
plot(freq_MHz, 20*log10(err_load+eps), 'LineWidth',1.5);
xlabel('Frequency (MHz)'); ylabel('|Error| (dB)');
title('LOAD:  |Corrected - Ideal|'); grid on;

sgtitle('Calibration Self-Verification (should be at noise floor)');

% =========================================================================
% Helper: load one AD8302 sweep file -> struct with freq_hz and gamma
% =========================================================================
function rfm = load_ad8302(fname)
    fid = fopen(fname, 'r');
    if fid == -1, error('Cannot open file: %s', fname); end
    rfm_raw = [];
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line) || strncmp(line, '===', 3) || strncmp(line, 'freq', 4), continue; end
        vals = sscanf(line, '%f,%f,%f,%f,%f');
        if numel(vals) >= 3
            rfm_raw(end+1, :) = vals(1:3)';
        end
    end
    fclose(fid);
    rfm.freq_hz = rfm_raw(:, 1);
    mag_linear  = 10 .^ (rfm_raw(:, 2) / 20);
    phase_deg   = correct_phase(rfm_raw(:, 3));   % fix sign ambiguities before converting
    phase_rad   = deg2rad(phase_deg);
    rfm.gamma   = mag_linear .* exp(1j * phase_rad);
    fprintf('Loaded %-35s  %d points  (%.0f – %.0f MHz)\n', ...
        fname, numel(rfm.freq_hz), rfm.freq_hz(1)/1e6, rfm.freq_hz(end)/1e6);
end

function ph_corr = correct_phase(ph)
    ph_corr = ph;
    n = numel(ph);
    for i = 1:n
        if i == 1
            slope = ph(2) - ph(1);
        elseif i == n
            slope = ph(n) - ph(n-1);
        else
            slope = ph(i+1) - ph(i-1);   % central difference
        end
        if slope > 0
            ph_corr(i) = -ph(i);
        end
    end
end