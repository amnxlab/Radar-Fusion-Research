classdef SwerlingIITarget < handle
    % SwerlingIITarget - Generate pulse-to-pulse fluctuating RCS (Swerling Type II)
    %
    % Implements Swerling Type II (Case 2) model:
    % - RCS fluctuates independently for each pulse/chirp
    % - Chi-squared distribution with 2 degrees of freedom (exponential)
    % - Complex amplitudes are IID circular Gaussian
    %
    % Key Properties:
    %   MeanRCS_dBsm - Mean RCS in dBsm
    %   MeanRCS_m2   - Mean RCS in m² (linear)
    %   NumChirps    - Number of chirps (M) to generate
    %
    % Usage:
    %   target = SwerlingIITarget(-17, 512, 5.8e9);
    %   [alpha, rcs_m2] = target.generateFluctuatingRCS();
    %   target.plotRCSPattern();
    %   target.plotHistogram();
    %
    % Reference:
    %   Swerling, P. (1960). "Probability of Detection for Fluctuating Targets"
    %   MathWorks: https://www.mathworks.com/help/phased/ug/swerling-2-target-models.html
    %
    % Author: Radar Fusion Research Project
    % Date: February 2026
    
    properties
        MeanRCS_dBsm    % Mean RCS in dBsm
        MeanRCS_m2      % Mean RCS in m² (linear scale)
        NumChirps       % Number of chirps/pulses (M)
        OperatingFreq   % Operating frequency in Hz
        SeedMode        % 'shuffle' or fixed seed value
    end
    
    properties (SetAccess = private)
        ComplexAmplitudes   % [M×1] Complex scattering amplitudes (α_m)
        RCS_m2              % [M×1] Instantaneous RCS in m²
        RCS_dB              % [M×1] Instantaneous RCS in dB relative to mean
        LocalSNR_dB         % [M×1] Local SNR deviation from mean (dB)
        IsGenerated         % Flag indicating if RCS has been generated
    end
    
    methods
        function obj = SwerlingIITarget(meanRCS_dBsm, numChirps, fc, seedMode)
            % Constructor for SwerlingIITarget
            %
            % Inputs:
            %   meanRCS_dBsm - Mean RCS in dBsm (e.g., -17 for 5.8 GHz UAV)
            %   numChirps    - Number of chirps/pulses (e.g., 512)
            %   fc           - Operating frequency in Hz (e.g., 5.8e9)
            %   seedMode     - 'shuffle' or integer seed for reproducibility
            
            if nargin < 4
                seedMode = 'shuffle';
            end
            if nargin < 3
                fc = 5.8e9;
            end
            if nargin < 2
                numChirps = 512;
            end
            if nargin < 1
                meanRCS_dBsm = -17;
            end
            
            obj.MeanRCS_dBsm = meanRCS_dBsm;
            obj.MeanRCS_m2 = 10^(meanRCS_dBsm / 10);
            obj.NumChirps = numChirps;
            obj.OperatingFreq = fc;
            obj.SeedMode = seedMode;
            obj.IsGenerated = false;
            
            % Initialize RNG
            if ischar(seedMode) || isstring(seedMode)
                if strcmpi(seedMode, 'shuffle')
                    rng('shuffle');
                end
            else
                rng(seedMode);
            end
        end
        
        function [alpha, rcs_m2] = generateFluctuatingRCS(obj)
            % Generate M independent complex amplitudes with Swerling II statistics
            %
            % Outputs:
            %   alpha  - [M×1] Complex scattering amplitudes
            %   rcs_m2 - [M×1] Instantaneous RCS values in m²
            %
            % The complex amplitude is modeled as:
            %   α_m = sqrt(σ_m) * exp(j*θ_m)
            % where σ_m ~ Exponential(mean_σ) and θ_m ~ Uniform(0, 2π)
            %
            % Equivalently, using complex Gaussian:
            %   α_m = X_m + j*Y_m, where X_m, Y_m ~ N(0, mean_σ/2)
            
            M = obj.NumChirps;
            mean_sigma = obj.MeanRCS_m2;
            
            % Method: Complex Gaussian with variance = mean_RCS/2 per I/Q component
            % This ensures |α|² ~ Exponential(mean_sigma) = Chi-squared(2 DOF)
            variance_per_component = mean_sigma / 2;
            std_per_component = sqrt(variance_per_component);
            
            % IID complex Gaussian random variables
            X = std_per_component * randn(M, 1);
            Y = std_per_component * randn(M, 1);
            alpha = X + 1j * Y;
            
            % Instantaneous RCS is magnitude squared
            rcs_m2 = abs(alpha).^2;
            
            % Store results
            obj.ComplexAmplitudes = alpha;
            obj.RCS_m2 = rcs_m2;
            obj.RCS_dB = 10*log10(rcs_m2 / mean_sigma);  % dB relative to mean
            obj.LocalSNR_dB = 10*log10(rcs_m2 / mean_sigma);  % Same as RCS_dB for point target
            obj.IsGenerated = true;
        end
        
        function [afs_data] = getAFSData(obj, baseline_snr_dB)
            % Get data structure for Amplitude Fluctuation Suppression (AFS) stage
            %
            % Input:
            %   baseline_snr_dB - Baseline SNR in dB (mean SNR at mean RCS)
            %
            % Output:
            %   afs_data - Structure containing:
            %     .rcs_m2         - [M×1] Instantaneous RCS in m²
            %     .rcs_dB         - [M×1] RCS in dB relative to mean
            %     .complex_alpha  - [M×1] Complex scattering amplitudes
            %     .local_snr_dB   - [M×1] Local SNR in dB
            %     .low_snr_mask   - [M×1] Logical mask for low-SNR pulses
            
            if ~obj.IsGenerated
                obj.generateFluctuatingRCS();
            end
            
            if nargin < 2
                baseline_snr_dB = 0;  % Assume 0 dB baseline if not specified
            end
            
            % Local SNR = baseline + RCS deviation from mean
            local_snr_dB = baseline_snr_dB + obj.RCS_dB;
            
            % Default threshold: 6 dB below baseline (aggressive nulling)
            threshold_dB = baseline_snr_dB - 6;
            
            afs_data = struct();
            afs_data.rcs_m2 = obj.RCS_m2;
            afs_data.rcs_dB = obj.RCS_dB;
            afs_data.complex_alpha = obj.ComplexAmplitudes;
            afs_data.local_snr_dB = local_snr_dB;
            afs_data.low_snr_mask = local_snr_dB < threshold_dB;
            afs_data.threshold_dB = threshold_dB;
            afs_data.pct_low_snr = 100 * sum(afs_data.low_snr_mask) / obj.NumChirps;
        end
        
        function fig = plotRCSPattern(obj, varargin)
            % Plot RCS fluctuation pattern over chirps
            %
            % Optional Name-Value pairs:
            %   'Units' - 'dBsm' (default) or 'm2'
            %   'Title' - Custom title string
            
            p = inputParser;
            addParameter(p, 'Units', 'dBsm', @ischar);
            addParameter(p, 'Title', '', @ischar);
            parse(p, varargin{:});
            
            if ~obj.IsGenerated
                obj.generateFluctuatingRCS();
            end
            
            fig = figure('Name', 'Swerling II RCS Pattern', 'Color', 'w');
            
            chirp_idx = 1:obj.NumChirps;
            
            if strcmpi(p.Results.Units, 'dBsm')
                rcs_plot = 10*log10(obj.RCS_m2);
                mean_line = obj.MeanRCS_dBsm;
                ylabel_str = 'RCS (dBsm)';
            else
                rcs_plot = obj.RCS_m2;
                mean_line = obj.MeanRCS_m2;
                ylabel_str = 'RCS (m²)';
            end
            
            % Main RCS plot
            subplot(2,1,1);
            plot(chirp_idx, rcs_plot, 'b-', 'LineWidth', 0.8);
            hold on;
            yline(mean_line, 'r--', 'LineWidth', 1.5, 'Label', 'Mean RCS');
            hold off;
            grid on;
            xlabel('Chirp Index (m)');
            ylabel(ylabel_str);
            
            if isempty(p.Results.Title)
                title(sprintf('Swerling II RCS Fluctuation (M=%d, Mean=%.1f dBsm, f_c=%.1f GHz)', ...
                    obj.NumChirps, obj.MeanRCS_dBsm, obj.OperatingFreq/1e9));
            else
                title(p.Results.Title);
            end
            
            % Deviation from mean (dB)
            subplot(2,1,2);
            plot(chirp_idx, obj.RCS_dB, 'b-', 'LineWidth', 0.8);
            hold on;
            yline(0, 'r--', 'LineWidth', 1.5);
            yline(-10, 'k:', 'LineWidth', 1, 'Label', '-10 dB');
            yline(10, 'k:', 'LineWidth', 1, 'Label', '+10 dB');
            hold off;
            grid on;
            xlabel('Chirp Index (m)');
            ylabel('RCS Deviation (dB)');
            title('RCS Fluctuation Relative to Mean');
            
            % Statistics annotation
            rcs_range = max(obj.RCS_dB) - min(obj.RCS_dB);
            annotation('textbox', [0.15, 0.35, 0.3, 0.08], ...
                'String', sprintf('Range: %.1f dB\nStd: %.2f dB', rcs_range, std(obj.RCS_dB)), ...
                'FitBoxToText', 'on', 'BackgroundColor', 'w', 'EdgeColor', 'k');
        end
        
        function fig = plotHistogram(obj, varargin)
            % Plot histogram of RCS with theoretical chi-squared overlay
            %
            % Optional Name-Value pairs:
            %   'NumBins' - Number of histogram bins (default: 30)
            
            p = inputParser;
            addParameter(p, 'NumBins', 30, @isnumeric);
            parse(p, varargin{:});
            
            if ~obj.IsGenerated
                obj.generateFluctuatingRCS();
            end
            
            fig = figure('Name', 'Swerling II RCS Histogram', 'Color', 'w');
            
            % Histogram in m² (linear scale)
            subplot(1,2,1);
            histogram(obj.RCS_m2, p.Results.NumBins, 'Normalization', 'pdf', ...
                'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'w');
            hold on;
            
            % Theoretical PDF: Exponential with mean = MeanRCS_m2
            x_theory = linspace(0, max(obj.RCS_m2)*1.2, 200);
            pdf_theory = (1/obj.MeanRCS_m2) * exp(-x_theory / obj.MeanRCS_m2);
            plot(x_theory, pdf_theory, 'r-', 'LineWidth', 2);
            
            hold off;
            grid on;
            xlabel('RCS (m²)');
            ylabel('Probability Density');
            title('RCS Distribution (Linear Scale)');
            legend('Simulated', 'Chi-squared(2) Theory', 'Location', 'best');
            
            % Histogram in dBsm
            subplot(1,2,2);
            rcs_dBsm = 10*log10(obj.RCS_m2);
            histogram(rcs_dBsm, p.Results.NumBins, 'Normalization', 'pdf', ...
                'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'w');
            hold on;
            
            % Theoretical PDF in dB scale
            % If σ ~ Exp(μ), then σ_dB = 10*log10(σ) has PDF:
            % p(σ_dB) = (ln(10)/10) * (1/μ) * exp(σ_dB*ln(10)/10) * exp(-10^(σ_dB/10)/μ)
            x_dB = linspace(min(rcs_dBsm)-5, max(rcs_dBsm)+5, 200);
            x_linear = 10.^(x_dB/10);
            pdf_dB = (log(10)/10) * (x_linear / obj.MeanRCS_m2) .* ...
                     exp(-x_linear / obj.MeanRCS_m2);
            plot(x_dB, pdf_dB, 'r-', 'LineWidth', 2);
            
            xline(obj.MeanRCS_dBsm, 'g--', 'LineWidth', 1.5, 'Label', 'Mean');
            
            hold off;
            grid on;
            xlabel('RCS (dBsm)');
            ylabel('Probability Density');
            title('RCS Distribution (dB Scale)');
            legend('Simulated', 'Theory', 'Location', 'best');
            
            sgtitle(sprintf('Swerling II Target Return vs RCS (M=%d)', obj.NumChirps));
        end
        
        function [h, p_value] = validateDistribution(obj)
            % Validate that generated RCS follows chi-squared(2) distribution
            %
            % Outputs:
            %   h       - 0 if null hypothesis accepted (good fit), 1 if rejected
            %   p_value - p-value from chi-squared goodness-of-fit test
            
            if ~obj.IsGenerated
                obj.generateFluctuatingRCS();
            end
            
            % Use Kolmogorov-Smirnov test against exponential distribution
            % Exponential(λ) with λ = 1/mean is equivalent to chi-squared(2)
            [h, p_value] = kstest(obj.RCS_m2, 'CDF', makedist('Exponential', 'mu', obj.MeanRCS_m2));
            
            if h == 0
                fprintf('✓ Distribution validation PASSED (p=%.4f)\n', p_value);
                fprintf('  RCS follows chi-squared(2) / Exponential distribution\n');
            else
                fprintf('✗ Distribution validation FAILED (p=%.4f)\n', p_value);
                fprintf('  RCS may not follow expected Swerling II statistics\n');
            end
        end
        
        function stats = getStatistics(obj)
            % Get summary statistics of generated RCS
            %
            % Output:
            %   stats - Structure with mean, std, min, max, etc.
            
            if ~obj.IsGenerated
                obj.generateFluctuatingRCS();
            end
            
            stats = struct();
            stats.mean_m2 = mean(obj.RCS_m2);
            stats.mean_dBsm = 10*log10(stats.mean_m2);
            stats.std_m2 = std(obj.RCS_m2);
            stats.std_dB = std(obj.RCS_dB);
            stats.min_dBsm = 10*log10(min(obj.RCS_m2));
            stats.max_dBsm = 10*log10(max(obj.RCS_m2));
            stats.range_dB = stats.max_dBsm - stats.min_dBsm;
            stats.median_dBsm = 10*log10(median(obj.RCS_m2));
            
            % Expected theoretical values
            stats.theory_mean_m2 = obj.MeanRCS_m2;
            stats.theory_std_m2 = obj.MeanRCS_m2;  % For exponential, std = mean
            stats.mean_error_pct = 100 * abs(stats.mean_m2 - obj.MeanRCS_m2) / obj.MeanRCS_m2;
        end
        
        function printSummary(obj)
            % Print summary of target model and generated RCS
            
            fprintf('\n========== Swerling II Target Summary ==========\n');
            fprintf('Operating Frequency: %.2f GHz\n', obj.OperatingFreq/1e9);
            fprintf('Mean RCS (specified): %.2f dBsm (%.4f m²)\n', obj.MeanRCS_dBsm, obj.MeanRCS_m2);
            fprintf('Number of Chirps: %d\n', obj.NumChirps);
            
            if obj.IsGenerated
                stats = obj.getStatistics();
                fprintf('\n--- Generated RCS Statistics ---\n');
                fprintf('Mean (actual):  %.2f dBsm (%.4f m²) [Error: %.2f%%]\n', ...
                    stats.mean_dBsm, stats.mean_m2, stats.mean_error_pct);
                fprintf('Std Dev:        %.2f dB\n', stats.std_dB);
                fprintf('Min:            %.2f dBsm\n', stats.min_dBsm);
                fprintf('Max:            %.2f dBsm\n', stats.max_dBsm);
                fprintf('Range:          %.2f dB\n', stats.range_dB);
            else
                fprintf('\n[RCS not yet generated - call generateFluctuatingRCS()]\n');
            end
            fprintf('================================================\n\n');
        end
    end
    
    methods (Static)
        function demo()
            % Static demo method showing basic usage
            fprintf('=== Swerling II Target Demo ===\n\n');
            
            % Create target for 5.8 GHz band
            target_5p8 = SwerlingIITarget(-17, 512, 5.8e9, 42);
            target_5p8.generateFluctuatingRCS();
            target_5p8.printSummary();
            target_5p8.validateDistribution();
            
            % Create target for 24 GHz band
            target_24 = SwerlingIITarget(-9.5, 512, 24e9, 42);
            target_24.generateFluctuatingRCS();
            target_24.printSummary();
            target_24.validateDistribution();
            
            fprintf('\n=== SNR Offset Verification ===\n');
            snr_offset = target_24.MeanRCS_dBsm - target_5p8.MeanRCS_dBsm;
            fprintf('RCS difference (24 GHz - 5.8 GHz): %.2f dB\n', snr_offset);
            lambda_ratio = (3e8/24e9) / (3e8/5.8e9);
            lambda_offset = 20*log10(lambda_ratio);
            fprintf('Wavelength scaling (λ² term): %.2f dB\n', lambda_offset);
            fprintf('Net SNR offset: %.2f dB\n', snr_offset + lambda_offset);
        end
    end
end
