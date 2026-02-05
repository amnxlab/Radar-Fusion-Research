% run_swerling_demo.m - Demonstration and Visualization of Swerling II Target Models
%
% This script generates all required plots for the Swerling II implementation:
% 1. RCS fluctuation pattern over M=512 chirps (both bands)
% 2. Histogram of Swerling II target returns vs RCS (m²)
% 3. Comparison with theoretical chi-squared(2) distribution
% 4. RCS patterns vs elevation angles (drone model)
% 5. Extended scatterer pattern visualization (if applicable)
%
% Usage:
%   run_swerling_demo          % Run with default settings
%   run_swerling_demo(true)    % Save figures to files
%
% Author: Radar Fusion Research Project
% Date: February 2026

function run_swerling_demo(saveFigures)
    if nargin < 1
        saveFigures = false;
    end
    
    % Ensure we're in the correct directory or add path
    thisDir = fileparts(mfilename('fullpath'));
    addpath(thisDir);
    
    % Create output directory for figures if saving
    if saveFigures
        figDir = fullfile(thisDir, '..', 'docs', 'figures');
        if ~exist(figDir, 'dir')
            mkdir(figDir);
        end
    end
    
    fprintf('========================================\n');
    fprintf(' Swerling II Target Model Demonstration\n');
    fprintf('========================================\n\n');
    
    % Parameters matching the radar models
    M = 512;  % Number of chirps
    seed = 42;  % Fixed seed for reproducibility
    
    % Band 1: 5.8 GHz
    fc_1 = 5.8e9;
    mean_rcs_dBsm_1 = -17;  % dBsm
    
    % Band 2: 24 GHz
    fc_2 = 24e9;
    mean_rcs_dBsm_2 = -9.5;  % dBsm
    
    %% ==================== Section 1: Basic Swerling II Targets ====================
    fprintf('Section 1: Creating Swerling II targets for both bands...\n');
    
    % Create targets
    target_5p8 = SwerlingIITarget(mean_rcs_dBsm_1, M, fc_1, seed);
    target_24 = SwerlingIITarget(mean_rcs_dBsm_2, M, fc_2, seed + 1);
    
    % Generate fluctuating RCS
    [alpha_5p8, rcs_5p8] = target_5p8.generateFluctuatingRCS();
    [alpha_24, rcs_24] = target_24.generateFluctuatingRCS();
    
    % Print summaries
    target_5p8.printSummary();
    target_24.printSummary();
    
    %% ==================== Section 2: RCS Pattern Plots ====================
    fprintf('\nSection 2: Generating RCS pattern plots...\n');
    
    % Individual pattern plots
    fig1 = target_5p8.plotRCSPattern('Title', 'Swerling II RCS Pattern - 5.8 GHz Band');
    fig2 = target_24.plotRCSPattern('Title', 'Swerling II RCS Pattern - 24 GHz Band');
    
    % Combined dual-band comparison
    fig3 = figure('Name', 'Dual-Band RCS Comparison', 'Color', 'w', ...
        'Position', [100, 100, 1200, 600]);
    
    subplot(2,2,1);
    plot(1:M, 10*log10(rcs_5p8), 'b-', 'LineWidth', 0.8);
    hold on;
    yline(mean_rcs_dBsm_1, 'r--', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('Chirp Index');
    ylabel('RCS (dBsm)');
    title(sprintf('5.8 GHz Band (Mean = %.1f dBsm)', mean_rcs_dBsm_1));
    ylim([mean_rcs_dBsm_1 - 30, mean_rcs_dBsm_1 + 15]);
    
    subplot(2,2,2);
    plot(1:M, 10*log10(rcs_24), 'r-', 'LineWidth', 0.8);
    hold on;
    yline(mean_rcs_dBsm_2, 'b--', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('Chirp Index');
    ylabel('RCS (dBsm)');
    title(sprintf('24 GHz Band (Mean = %.1f dBsm)', mean_rcs_dBsm_2));
    ylim([mean_rcs_dBsm_2 - 30, mean_rcs_dBsm_2 + 15]);
    
    subplot(2,2,3:4);
    plot(1:M, target_5p8.RCS_dB, 'b-', 'LineWidth', 0.8); hold on;
    plot(1:M, target_24.RCS_dB, 'r-', 'LineWidth', 0.8);
    yline(0, 'k--', 'LineWidth', 1);
    yline(-10, 'k:', 'LineWidth', 0.5);
    yline(10, 'k:', 'LineWidth', 0.5);
    hold off;
    grid on;
    xlabel('Chirp Index');
    ylabel('RCS Deviation from Mean (dB)');
    title('Normalized RCS Fluctuation - Both Bands');
    legend('5.8 GHz', '24 GHz', 'Location', 'best');
    
    sgtitle('Swerling II Target RCS Patterns - Dual Band Comparison');
    
    if saveFigures
        saveas(fig1, fullfile(figDir, 'rcs_pattern_5p8ghz.png'));
        saveas(fig2, fullfile(figDir, 'rcs_pattern_24ghz.png'));
        saveas(fig3, fullfile(figDir, 'rcs_pattern_dual_band.png'));
    end
    
    %% ==================== Section 3: Histogram Plots ====================
    fprintf('\nSection 3: Generating histogram plots...\n');
    
    % Individual histograms with theoretical overlay
    fig4 = target_5p8.plotHistogram('NumBins', 40);
    sgtitle('Swerling II Histogram - 5.8 GHz');
    
    fig5 = target_24.plotHistogram('NumBins', 40);
    sgtitle('Swerling II Histogram - 24 GHz');
    
    % Combined histogram comparison
    fig6 = figure('Name', 'RCS Histogram Comparison', 'Color', 'w', ...
        'Position', [100, 100, 1000, 500]);
    
    subplot(1,2,1);
    % Normalize both to same scale for comparison
    rcs_norm_5p8 = rcs_5p8 / target_5p8.MeanRCS_m2;
    rcs_norm_24 = rcs_24 / target_24.MeanRCS_m2;
    
    histogram(rcs_norm_5p8, 30, 'Normalization', 'pdf', ...
        'FaceColor', [0.2 0.4 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    hold on;
    histogram(rcs_norm_24, 30, 'Normalization', 'pdf', ...
        'FaceColor', [0.8 0.3 0.2], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    
    % Theoretical chi-squared(2) / Exponential(1) PDF
    x = linspace(0, max([rcs_norm_5p8; rcs_norm_24]), 200);
    pdf_theory = exp(-x);  % Exponential with mean=1
    plot(x, pdf_theory, 'k-', 'LineWidth', 2);
    
    hold off;
    grid on;
    xlabel('Normalized RCS (\sigma / \sigma_{mean})');
    ylabel('Probability Density');
    title('Normalized RCS Distribution');
    legend('5.8 GHz', '24 GHz', 'Chi-squared(2) Theory', 'Location', 'best');
    
    subplot(1,2,2);
    % Q-Q plot against exponential
    sorted_rcs = sort(rcs_norm_5p8);
    n = length(sorted_rcs);
    theoretical_quantiles = -log(1 - ((1:n) - 0.5) / n);
    
    plot(theoretical_quantiles, sorted_rcs, 'b.', 'MarkerSize', 8);
    hold on;
    plot([0, max(theoretical_quantiles)], [0, max(theoretical_quantiles)], 'r--', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('Theoretical Quantiles (Exponential)');
    ylabel('Sample Quantiles');
    title('Q-Q Plot: 5.8 GHz vs Exponential');
    legend('Data', 'Perfect Fit', 'Location', 'best');
    
    sgtitle('Statistical Validation of Swerling II Model');
    
    if saveFigures
        saveas(fig4, fullfile(figDir, 'histogram_5p8ghz.png'));
        saveas(fig5, fullfile(figDir, 'histogram_24ghz.png'));
        saveas(fig6, fullfile(figDir, 'histogram_comparison.png'));
    end
    
    %% ==================== Section 4: Distribution Validation ====================
    fprintf('\nSection 4: Statistical validation...\n');
    
    fprintf('\n--- 5.8 GHz Band ---\n');
    [h1, p1] = target_5p8.validateDistribution();
    
    fprintf('\n--- 24 GHz Band ---\n');
    [h2, p2] = target_24.validateDistribution();
    
    %% ==================== Section 5: Drone RCS Model ====================
    fprintf('\nSection 5: Drone RCS Model with elevation patterns...\n');
    
    % Create drone models
    drone_5p8 = DroneRCSModel(mean_rcs_dBsm_1, M, fc_1, seed, ...
        'DroneType', 'quadrotor', 'NumScatterers', 5);
    drone_24 = DroneRCSModel(mean_rcs_dBsm_2, M, fc_2, seed + 1, ...
        'DroneType', 'quadrotor', 'NumScatterers', 5);
    
    % Generate RCS
    drone_5p8.generateFluctuatingRCS();
    drone_24.generateFluctuatingRCS();
    
    % Plot extended scatterer pattern
    fig7 = drone_5p8.plotExtendedScattererPattern();
    sgtitle('Extended Drone Scatterer Pattern - 5.8 GHz');
    
    % Plot RCS vs elevation
    fig8 = drone_5p8.plotRCSvsElevation();
    fig9 = drone_24.plotRCSvsElevation();
    
    if saveFigures
        saveas(fig7, fullfile(figDir, 'extended_scatterer_pattern.png'));
        saveas(fig8, fullfile(figDir, 'rcs_vs_elevation_5p8ghz.png'));
        saveas(fig9, fullfile(figDir, 'rcs_vs_elevation_24ghz.png'));
    end
    
    %% ==================== Section 6: AFS Output Data ====================
    fprintf('\nSection 6: AFS (Amplitude Fluctuation Suppression) output...\n');
    
    % Get AFS data for both bands
    % SNR values from the radar models
    snr_5p8_dB = 0.11;  % R1 baseline SNR
    snr_24_dB = -4;     % R2 baseline SNR (lower due to noise figure)
    
    afs_5p8 = target_5p8.getAFSData(snr_5p8_dB);
    afs_24 = target_24.getAFSData(snr_24_dB);
    
    fprintf('\n--- AFS Data for 5.8 GHz ---\n');
    fprintf('Baseline SNR: %.2f dB\n', snr_5p8_dB);
    fprintf('Threshold: %.2f dB\n', afs_5p8.threshold_dB);
    fprintf('Low-SNR pulses: %.1f%% (%d of %d)\n', ...
        afs_5p8.pct_low_snr, sum(afs_5p8.low_snr_mask), M);
    
    fprintf('\n--- AFS Data for 24 GHz ---\n');
    fprintf('Baseline SNR: %.2f dB\n', snr_24_dB);
    fprintf('Threshold: %.2f dB\n', afs_24.threshold_dB);
    fprintf('Low-SNR pulses: %.1f%% (%d of %d)\n', ...
        afs_24.pct_low_snr, sum(afs_24.low_snr_mask), M);
    
    % Plot local SNR with mask
    fig10 = figure('Name', 'AFS Local SNR Analysis', 'Color', 'w', ...
        'Position', [100, 100, 1200, 400]);
    
    subplot(1,2,1);
    plot(1:M, afs_5p8.local_snr_dB, 'b-', 'LineWidth', 0.8);
    hold on;
    yline(afs_5p8.threshold_dB, 'r--', 'LineWidth', 1.5, 'Label', 'Threshold');
    scatter(find(afs_5p8.low_snr_mask), afs_5p8.local_snr_dB(afs_5p8.low_snr_mask), ...
        20, 'r', 'filled');
    hold off;
    grid on;
    xlabel('Chirp Index');
    ylabel('Local SNR (dB)');
    title(sprintf('5.8 GHz - %.1f%% below threshold', afs_5p8.pct_low_snr));
    legend('Local SNR', 'Threshold', 'Low-SNR pulses', 'Location', 'best');
    
    subplot(1,2,2);
    plot(1:M, afs_24.local_snr_dB, 'r-', 'LineWidth', 0.8);
    hold on;
    yline(afs_24.threshold_dB, 'b--', 'LineWidth', 1.5, 'Label', 'Threshold');
    scatter(find(afs_24.low_snr_mask), afs_24.local_snr_dB(afs_24.low_snr_mask), ...
        20, 'b', 'filled');
    hold off;
    grid on;
    xlabel('Chirp Index');
    ylabel('Local SNR (dB)');
    title(sprintf('24 GHz - %.1f%% below threshold', afs_24.pct_low_snr));
    legend('Local SNR', 'Threshold', 'Low-SNR pulses', 'Location', 'best');
    
    sgtitle('AFS Local SNR Analysis - Pulse Nullification Candidates');
    
    if saveFigures
        saveas(fig10, fullfile(figDir, 'afs_local_snr.png'));
    end
    
    %% ==================== Section 7: SNR Offset Verification ====================
    fprintf('\nSection 7: Dual-band SNR offset verification...\n');
    
    % Calculate expected SNR offset
    lambda_5p8 = 3e8 / fc_1;
    lambda_24 = 3e8 / fc_2;
    
    rcs_offset_dB = mean_rcs_dBsm_2 - mean_rcs_dBsm_1;
    lambda_offset_dB = 20 * log10(lambda_24 / lambda_5p8);
    net_snr_offset = rcs_offset_dB + lambda_offset_dB;
    
    fprintf('\n--- SNR Offset Analysis ---\n');
    fprintf('RCS difference (24 GHz - 5.8 GHz): %.2f dB\n', rcs_offset_dB);
    fprintf('Wavelength scaling (λ² term):     %.2f dB\n', lambda_offset_dB);
    fprintf('Net SNR offset:                   %.2f dB\n', net_snr_offset);
    fprintf('Expected offset:                  ~4.85 dB\n');
    
    %% ==================== Summary ====================
    fprintf('\n========================================\n');
    fprintf(' Demonstration Complete\n');
    fprintf('========================================\n');
    
    if saveFigures
        fprintf('Figures saved to: %s\n', figDir);
    end
    
    fprintf('\nGenerated plots:\n');
    fprintf('  1. RCS Pattern - 5.8 GHz\n');
    fprintf('  2. RCS Pattern - 24 GHz\n');
    fprintf('  3. Dual-Band RCS Comparison\n');
    fprintf('  4. Histogram - 5.8 GHz\n');
    fprintf('  5. Histogram - 24 GHz\n');
    fprintf('  6. Statistical Validation\n');
    fprintf('  7. Extended Scatterer Pattern\n');
    fprintf('  8. RCS vs Elevation - 5.8 GHz\n');
    fprintf('  9. RCS vs Elevation - 24 GHz\n');
    fprintf(' 10. AFS Local SNR Analysis\n');
    
    fprintf('\nTo use in R1_model.m or R2_model.m:\n');
    fprintf('  addpath(''targets'');\n');
    fprintf('  target = SwerlingIITarget(-17, NumSweeps, fc);\n');
    fprintf('  [alpha, rcs] = target.generateFluctuatingRCS();\n');
end

% Run demo if called directly
if ~isdeployed && ~(exist('run_swerling_demo', 'var') && islogical(run_swerling_demo))
    % Check if running as script
    stack = dbstack;
    if length(stack) == 1
        run_swerling_demo(false);
    end
end
