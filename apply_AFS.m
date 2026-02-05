function [cleaned_signal, afs_metrics] = apply_AFS(beat_matrix, params)
% APPLY_AFS - Amplitude Fluctuation Suppression for FMCW radar
%
% Implements a modular AFS signal processing stage with CFAR-like sliding
% window detection to identify and nullify low-SNR pulses before non-coherent
% integration.
%
% Algorithm:
%   1. Compute range profile via FFT for each chirp
%   2. For each chirp, use sliding window with protection/reference cells
%   3. If signal(CUT) <= mean(reference), nullify the pulse
%   4. Non-coherent integration of remaining high-SNR pulses
%
% Inputs:
%   beat_matrix - [Nsweep × M] complex beat signal (dechirped)
%   params      - Structure with:
%     .range_estimate - Estimated target range from coarse detector (m)
%                       NOTE: Pass R_est from upstream, NOT ground truth!
%     .delta_R        - Range resolution (m)
%     .v_r_max        - Max radial velocity (m/s) [default: 10]
%     .T_r            - Chirp repetition interval (s)
%     .n_ref          - Number of reference cells per side [default: 8]
%     .fs             - Sampling frequency (Hz)
%     .plot_enable    - Enable visualization plots [default: true]

%
% Outputs:
%   cleaned_signal - [Nsweep × 1] non-coherently integrated amplitude
%   afs_metrics    - Structure with processing metrics:
%     .eta            - Rate of SNR fluctuation (removed/total)
%     .n_removed      - Number of nullified pulses
%     .n_total        - Total pulses
%     .beat_cleaned   - Cleaned beat matrix (low-SNR pulses zeroed)
%     .low_snr_mask   - Logical mask of removed pulses
%     .snr_gain_dB    - Theoretical SNR gain from integration
%
% Reference:
%   Dual-band FMCW radar simulation with Swerling II target model
%
% Author: Radar Fusion Research Project
% Date: February 2026

%% ===================== 1) Parameter Extraction & Defaults =====================
range_estimate = params.range_estimate;
delta_R = params.delta_R;
v_r_max = getfield_safe(params, 'v_r_max', 10);
T_r = params.T_r;
n_ref = getfield_safe(params, 'n_ref', 8);
fs = getfield_safe(params, 'fs', 1e6);
plot_enable = getfield_safe(params, 'plot_enable', true);


[Nsweep, M] = size(beat_matrix);

fprintf('\n========== AFS Processing ==========\n');
fprintf('Beat matrix size: %d samples × %d chirps\n', Nsweep, M);
fprintf('Target range estimate: %.2f m\n', range_estimate);
fprintf('Range resolution: %.3f m\n', delta_R);

%% ===================== 2) Calculate Protection Window =====================
% Protection cells to account for target mobility
% np = 2 * floor(v_r_max * T_r / delta_R) + 2
np = 2 * floor(v_r_max * T_r / delta_R) + 2;
np = max(np, 2);  % Minimum 2 protection cells

% Total window configuration: [nr + np + CUT + np + nr]
window_half = n_ref + np;

fprintf('Protection cells (np): %d\n', np);
fprintf('Reference cells (nr): %d per side\n', n_ref);
fprintf('Total window half-width: %d cells\n', window_half);

%% ===================== 3) Compute Range Profiles & Find CUT =====================
% FFT to get range profile per chirp
range_profiles = abs(fft(beat_matrix, [], 1));

% Create range axis
range_axis = (0:Nsweep-1) * (fs / Nsweep) * delta_R / (fs / Nsweep);
% More accurate: use beat frequency to range relationship
% For now, use simple linear mapping
range_bins_per_m = Nsweep * delta_R / (fs * T_r / 2);  % Approximate
range_axis = (0:Nsweep-1) * delta_R;

% Find range bin corresponding to target (Cell Under Test)
[~, cut_idx] = min(abs(range_axis - range_estimate));
cut_idx = max(1, min(Nsweep, cut_idx));

fprintf('Estimated range bin (CUT): %d (%.2f m)\n', cut_idx, range_axis(cut_idx));

%% ===================== 4) Sliding Window Detection Per Chirp =====================
low_snr_mask = false(M, 1);
ref_amplitudes = zeros(M, 1);
cut_amplitudes = zeros(M, 1);

for m = 1:M
    profile = range_profiles(:, m);
    
    % Get reference cell indices (excluding protection cells around CUT)
    ref_left_start = max(1, cut_idx - window_half);
    ref_left_end = max(1, cut_idx - np - 1);
    ref_right_start = min(Nsweep, cut_idx + np + 1);
    ref_right_end = min(Nsweep, cut_idx + window_half);
    
    % Build reference cell indices
    ref_cells = [];
    if ref_left_end >= ref_left_start
        ref_cells = [ref_cells, ref_left_start:ref_left_end];
    end
    if ref_right_end >= ref_right_start
        ref_cells = [ref_cells, ref_right_start:ref_right_end];
    end
    
    % Skip if no reference cells available
    if isempty(ref_cells)
        ref_amplitudes(m) = 0;
        cut_amplitudes(m) = profile(cut_idx);
        continue;
    end
    
    % Calculate reference amplitude (noise floor estimate)
    ref_avg = mean(profile(ref_cells));
    ref_amplitudes(m) = ref_avg;
    cut_amplitudes(m) = profile(cut_idx);
    
    % Threshold: if CUT <= noise floor, mark as low-SNR
    if profile(cut_idx) <= ref_avg
        low_snr_mask(m) = true;
    end
end

%% ===================== 5) Nullify Low-SNR Pulses =====================
beat_cleaned = beat_matrix;
beat_cleaned(:, low_snr_mask) = 0;

n_removed = sum(low_snr_mask);
n_remaining = M - n_removed;
eta = n_removed / M;

fprintf('\n--- AFS Detection Results ---\n');
fprintf('Low-SNR pulses identified: %d / %d (η = %.1f%%)\n', n_removed, M, eta * 100);
fprintf('Remaining high-SNR pulses: %d\n', n_remaining);

%% ===================== 6) Non-Coherent Integration =====================
% Sum power across remaining (high-SNR) chirps, then normalize
% Target power: stable (coherent sum of similar returns)
% Noise power: decreases (fewer pulses contributing)

if n_remaining > 0
    % Non-coherent integration: sqrt of average power
    power_sum = sum(abs(beat_cleaned).^2, 2);
    cleaned_signal = sqrt(power_sum / n_remaining);
    
    % Theoretical SNR gain from reducing noise contributions
    snr_gain_dB = 10 * log10(n_remaining);
else
    cleaned_signal = zeros(Nsweep, 1);
    snr_gain_dB = 0;
    warning('AFS: All pulses were nullified! Check threshold settings.');
end

fprintf('Non-coherent integration complete\n');
fprintf('Theoretical SNR gain: %.2f dB\n', snr_gain_dB);

%% ===================== 7) Visualization =====================
if plot_enable
    figure('Name', 'AFS Processing Results', 'Color', 'w', 'Position', [50, 50, 1500, 700]);
    
    % Plot 1: Range-Time Map BEFORE AFS
    subplot(2, 3, 1);
    imagesc(1:M, range_axis, mag2db(range_profiles + eps));
    axis xy; colorbar;
    hold on;
    yline(range_estimate, 'r--', 'LineWidth', 1.5);
    hold off;
    xlabel('Chirp Index');
    ylabel('Range (m)');
    title('Range-Time Map (Before AFS)');
    caxis([max(mag2db(range_profiles(:)))-60, max(mag2db(range_profiles(:)))]);
    
    % Plot 2: Range-Time Map AFTER AFS
    subplot(2, 3, 2);
    range_profiles_cleaned = abs(fft(beat_cleaned, [], 1));
    imagesc(1:M, range_axis, mag2db(range_profiles_cleaned + eps));
    axis xy; colorbar;
    hold on;
    yline(range_estimate, 'r--', 'LineWidth', 1.5);
    hold off;
    xlabel('Chirp Index');
    ylabel('Range (m)');
    title(sprintf('After AFS (η = %.1f%% removed)', eta * 100));
    caxis([max(mag2db(range_profiles(:)))-60, max(mag2db(range_profiles(:)))]);
    
    % Plot 3: Low-SNR pulse identification
    subplot(2, 3, 3);
    stem(1:M, low_snr_mask, 'r', 'LineWidth', 0.5, 'Marker', 'none');
    xlabel('Chirp Index');
    ylabel('Nullified (1) / Kept (0)');
    title(sprintf('Low-SNR Pulse Mask (η = %.1f%%)', eta * 100));
    ylim([-0.1, 1.1]);
    grid on;
    
    % Plot 4: CUT vs Reference amplitudes
    subplot(2, 3, 4);
    plot(1:M, mag2db(cut_amplitudes + eps), 'b-', 'LineWidth', 0.8);
    hold on;
    plot(1:M, mag2db(ref_amplitudes + eps), 'r--', 'LineWidth', 0.8);
    xlabel('Chirp Index');
    ylabel('Amplitude (dB)');
    title('CUT vs Reference Amplitude');
    legend('CUT (Target)', 'Reference (Noise)', 'Location', 'best');
    grid on;
    
    % Plot 5: Integrated range profile comparison
    subplot(2, 3, 5);
    % Before AFS (all chirps)
    integrated_before = sqrt(mean(abs(beat_matrix).^2, 2));
    plot(range_axis, mag2db(integrated_before + eps), 'b-', 'LineWidth', 1);
    hold on;
    plot(range_axis, mag2db(cleaned_signal + eps), 'r-', 'LineWidth', 1);
    xline(range_estimate, 'k--', 'LineWidth', 1);
    xlabel('Range (m)');
    ylabel('Amplitude (dB)');
    title('Integrated Range Profile');
    legend('Before AFS', 'After AFS', 'Estimate', 'Location', 'best');
    grid on;
    xlim([0, min(max(range_axis), range_estimate * 3)]);
    
    % Plot 6: SNR per chirp
    subplot(2, 3, 6);
    snr_per_chirp = cut_amplitudes ./ (ref_amplitudes + eps);
    snr_per_chirp_dB = mag2db(snr_per_chirp);
    plot(1:M, snr_per_chirp_dB, 'b-', 'LineWidth', 0.8);
    hold on;
    yline(0, 'r--', 'LineWidth', 1.5, 'Label', 'Threshold (0 dB)');
    % Mark removed pulses
    removed_idx = find(low_snr_mask);
    if ~isempty(removed_idx)
        scatter(removed_idx, snr_per_chirp_dB(removed_idx), 20, 'r', 'filled');
    end
    xlabel('Chirp Index');
    ylabel('Local SNR (dB)');
    title('Per-Chirp SNR Analysis');
    legend('SNR', 'Threshold', 'Removed', 'Location', 'best');
    grid on;
    
    sgtitle(sprintf('AFS Processing - %d chirps, η = %.1f%%, SNR gain = %.1f dB', M, eta*100, snr_gain_dB));
end

%% ===================== 8) Output Metrics Structure =====================
afs_metrics = struct();
afs_metrics.eta = eta;
afs_metrics.n_removed = n_removed;
afs_metrics.n_total = M;
afs_metrics.n_remaining = n_remaining;
afs_metrics.beat_cleaned = beat_cleaned;
afs_metrics.low_snr_mask = low_snr_mask;
afs_metrics.snr_gain_dB = snr_gain_dB;
afs_metrics.cut_amplitudes = cut_amplitudes;
afs_metrics.ref_amplitudes = ref_amplitudes;
afs_metrics.protection_cells = np;
afs_metrics.reference_cells = n_ref;
afs_metrics.cut_index = cut_idx;

fprintf('=====================================\n\n');

%% ===================== 9) Validation Metrics (Optional) =====================
% If ground truth is provided, calculate error metrics (EVAL ONLY, never used for detection)
if isfield(params, 'range_true') && isfinite(params.range_true)
    range_true = params.range_true;
    afs_metrics.range_error_m = abs(range_estimate - range_true);
    afs_metrics.range_error_bins = round(afs_metrics.range_error_m / delta_R);
    afs_metrics.range_estimate = range_estimate;
    afs_metrics.range_true = range_true;
    
    fprintf('\n--- Validation Metrics (EVAL ONLY) ---\n');
    fprintf('True range:      %.3f m\n', range_true);
    fprintf('Estimated range: %.3f m\n', range_estimate);
    fprintf('Error:           %.3f m (%.1f bins)\n', ...
            afs_metrics.range_error_m, afs_metrics.range_error_bins);
    
    % Check if CUT was misplaced
    true_bin = round(range_true / delta_R);
    if abs(cut_idx - true_bin) > np
        fprintf('⚠ WARNING: CUT placement error exceeds protection zone!\n');
    end
else
    % No ground truth provided (normal operational mode)
    afs_metrics.range_estimate = range_estimate;
end

end

%% ===================== Helper Function =====================
function val = getfield_safe(s, field, default)
% Safe field extraction with default value
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
