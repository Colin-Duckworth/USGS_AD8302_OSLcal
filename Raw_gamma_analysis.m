%% cable_compare.m
clc; clear all; close all;

% -------------------------------------------------------------------------
% File paths
% -------------------------------------------------------------------------

rfm_file_std = 'AD8302_raw_276pts.txt';                % measured reflection with normal cable
rfm_file_ext = 'AD8302_raw_276pts_extcable.txt';       % measured reflection with longer cable

rfm_file_OPEN  = 'AD8302_OPEN_52pts.txt';   % measured reflection open standard
rfm_file_SHORT = 'AD8302_SHORT_52pts.txt';  % measured reflection short standard
rfm_file_LOAD  = 'AD8302_LOAD_52pts.txt';   % measured reflection load standard

% =========================================================================
% Load both sweeps
% =========================================================================
std = load_ad8302(rfm_file_std);
ext = load_ad8302(rfm_file_ext);

f_std = std.freq_hz / 1e6;   % MHz
f_ext = ext.freq_hz / 1e6;

% =========================================================================
% Phase extraction + zero-offset correction
% Shift each dataset up by the magnitude of its minimum value so the
% minimum sits at 0 deg  (handles the ~-5 deg baseline offset)
% =========================================================================
ph_std_raw = rad2deg(angle(std.gamma));
ph_ext_raw = rad2deg(angle(ext.gamma));

ph_std = ph_std_raw - min(ph_std_raw);
ph_ext = ph_ext_raw - min(ph_ext_raw);

fprintf('STD phase offset applied: %.3f deg\n', -min(ph_std_raw));
fprintf('EXT phase offset applied: %.3f deg\n', -min(ph_ext_raw));

% =========================================================================
% Expected phase shift introduced by the additional cable (VNA-characterised)
% Empirical fit from VNA measurement: PH_e(f) = -0.775 * f_MHz
% Sign is negative: longer cable adds lag (phase decreases with frequency)
% =========================================================================
PH_e = 0.775 * f_std;   % deg, evaluated on standard cable frequency grid (MHz)

% =========================================================================
% Sign ambiguity resolution
% =========================================================================

PH_s = ph_std;   % measured phase, short cable,  [0,180] after offset shift
PH_l = ph_ext;   % measured phase, long cable,   [0,180] after offset shift

% --- Ambiguous folding function (maps any real angle -> [0, 180]) ---
ambig = @(phi) 180 - abs(mod(phi, 360) - 180);

% --- Wrapped phase distance on [0,180] ---
phase_dist = @(a, b) abs(a - b);   % both in [0,180], max distance is 180

% --- Predictions ---
P_nf = ambig( PH_s + PH_e);   % not-flipped prediction
P_f  = ambig(-PH_s + PH_e);   % flipped prediction

% --- Errors ---
Error_nf = phase_dist(PH_l, P_nf);
Error_f  = phase_dist(PH_l, P_f);

% --- Decision ---
IsFlipped = Error_f < Error_nf;   % logical array, true = flipped

% Margin threshold: if the winning hypothesis doesn't win by at least
% 10 degrees, the decision is unreliable — carry forward the previous
% point's value instead of making a new one
margin    = abs(Error_nf - Error_f);
confident = margin >= 10;

for i = 2:numel(IsFlipped)
    if ~confident(i)
        IsFlipped(i) = IsFlipped(i-1);   % carry forward
    end
end

% 7-point median filter to suppress any remaining rapid toggling
IsFlipped = logical(movmedian(double(IsFlipped), 7));

% --- Reconstructed true phase ---
% Where not flipped: true phase =  PH_s
% Where flipped:     true phase = -PH_s
% Result lives in [-180, 180]
ph_true = PH_s;
ph_true(IsFlipped) = -PH_s(IsFlipped);

% =========================================================================
% Continuity smoothness diagnostic
% =========================================================================
ph_candidate_nf = PH_s;
ph_candidate_f  = PH_s; ph_candidate_f(:) = -PH_s;

tv_true    = sum(abs(diff(ph_true)));
tv_nf_all  = sum(abs(diff(ph_candidate_nf)));
tv_f_all   = sum(abs(diff(ph_candidate_f)));

fprintf('\n--- Ambiguity resolution summary ---\n');
fprintf('  Points identified as flipped : %d / %d  (%.1f%%)\n', ...
    sum(IsFlipped), numel(IsFlipped), 100*mean(IsFlipped));
fprintf('  Total variation, reconstructed  ph_true : %.2f deg\n', tv_true);
fprintf('  Total variation, all-NF baseline         : %.2f deg\n', tv_nf_all);
fprintf('  Total variation, all-F  baseline         : %.2f deg\n', tv_f_all);
fprintf('  (lower total variation = smoother = more physically plausible)\n');

% =========================================================================
% Figure 1
% =========================================================================
figure(1); clf;

subplot(2,1,1);
plot(f_std, PH_s, 'b-', 'LineWidth', 1.5); hold on;
plot(f_ext, PH_l, 'r-', 'LineWidth', 1.5);
grid on;
ylabel('Phase (deg)');
title('Measured Phase: Short vs Long Cable');
legend('Short cable (PH\_s)', 'Long cable (PH\_l)', 'Location', 'best');
ylim([0 180]); yticks(0:30:180);

subplot(2,1,2);
plot(f_std, PH_s, 'b-', 'LineWidth', 1.5); hold on;

transitions = diff([0; IsFlipped; 0]);
starts = find(transitions ==  1);
ends   = find(transitions == -1) - 1;

for k = 1:numel(starts)
    x_patch = [f_std(starts(k)); f_std(ends(k)); f_std(ends(k)); f_std(starts(k))];
    y_patch = [0; 0; 180; 180];
    patch(x_patch, y_patch, [1 0.6 0.6], ...
        'FaceAlpha', 0.4, 'EdgeColor', 'none');
end

grid on;
ylabel('Phase (deg)');
xlabel('Frequency (MHz)');
title('Short Cable Phase  (shaded = IsFlipped)');
ylim([0 180]); yticks(0:30:180);

sgtitle('AD8302 Sign Ambiguity Resolution');

% =========================================================================
% Figure 2
% =========================================================================
figure(2); clf;

subplot(3,1,1);
plot(f_std, PH_s, 'b-', 'LineWidth', 1.5);
grid on;
ylabel('Phase (deg)');
title('Measured Short Cable Phase (PH\_s)');
ylim([0 180]); yticks(0:30:180);

subplot(3,1,2);
plot(f_std, PH_s, 'b-', 'LineWidth', 1.5); hold on;
plot(f_std, ph_true, 'r-', 'LineWidth', 1.5);
grid on;
ylabel('Phase (deg)');
title('Measured vs Sign-Corrected');
legend('PH\_s (measured)', 'ph\_true (corrected)', 'Location', 'best');
ylim([-180 180]); yticks(-180:30:180);

subplot(3,1,3);
plot(f_std, ph_true, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (MHz)');
ylabel('Phase (deg)');
title('Corrected Phase (ph\_true)');
ylim([-180 180]); yticks(-180:30:180);

sgtitle('Short Cable Phase: Ambiguity Correction');

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
    phase_rad   = deg2rad(rfm_raw(:, 3));
    rfm.gamma   = mag_linear .* exp(1j * phase_rad);
    fprintf('Loaded %s:  %d points  (%.0f – %.0f MHz)\n', ...
        fname, numel(rfm.freq_hz), rfm.freq_hz(1)/1e6, rfm.freq_hz(end)/1e6);
end