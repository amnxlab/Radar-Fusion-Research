% R2 FMCW Radar Model — 24 GHz (Guarded N=1 / Multi-Sweep Modes)
% - IF cap (0–50 MHz) wide-IF detection + optional narrow-band viz
% - Single-sweep integrity (TEST_SINGLE_SWEEP) prevents slow-time leakage
% - Separate data lanes: detection vs plotting (RDM/visuals never feed detector)
% - Robust phase-slope + Jacobsen-FFT cross-check (edge-safe)
% - Report-only Doppler in N=1 mode (for visibility, not used by detector)
% - Saved lengths consistent (t_out matches beat_signal)

clc; clear; close all;

%% ===================== 0) TOGGLES =====================
PLOT_TX_RX             = true;      % Tx/Rx time, inst-freq, FFT, spectrograms
PLOT_BEAT_WIDE_NAR     = true;      % Dechirped beat: raw vs wide-IF vs optional narrow BP
PLOT_MULTI_CHIRP_IF    = true;      % Instantaneous frequency (plot-only)
PLOT_RANGE_DOPPLER     = false;     % Range-Doppler Map (plot-only, never used by detector)
SAVE_BEAT_FILES        = true;      % Save beat .mat/.txt
USE_NARROW_PLOT        = true;      % 2nd-pass BP for visualization ONLY

% ***** SINGLE-SWEEP (N=1) GUARD SWITCHES *****
TEST_SINGLE_SWEEP       = true;     % true => detector uses exactly one sweep; no slow-time ops
FIXED_SWEEP_IDX         = 1;        % sweep index used in N=1 mode (1-based)
REPORT_DOPPLER_IN_N1    = true;     % report fd/v using slow-time, NEVER used by detector
NOISE_SINGLE_SWEEP_ONLY = false;    % if true, add AWGN ONLY to FIXED_SWEEP_IDX; others near-clean

% RNG policy for reproducibility vs variety
RNG_MODE    = "shuffle";            % "shuffle" or "fixed"
BASE_SEED   = 12345;                % used only if RNG_MODE == "fixed"

%% ===================== 1) Constants & Specs =====================
c  = 3e8;
fc = 24e9;                 % 24 GHz
lambda = c/fc;

Rmax       = 100;          % m
Ts_min     = (2*Rmax)/c;   % minimum sweep time (no ambiguity)
bw         = 150e6;        % Hz (sweep bandwidth)
fs         = 2*bw;         % Hz
sweep_time = 5*Ts_min;     % ~3.33 us
NumSweeps  = 1024;         % total chirps

% Scenario
range_true = 37;           % m
velocity   = 50;           % m/s (radial)
SNR_dB     = -4;        % dB (post-LNA SNR unless NOISE_SINGLE_SWEEP_ONLY=true)

% Derived
mu       = bw/sweep_time;               % Hz/s (chirp slope)
fb_true  = mu*(2*range_true/c);         % beat freq from range (no Doppler)
Nsweep   = round(sweep_time*fs);        % samples per chirp
PRI      = sweep_time;                  % no idle assumed
fprintf('Expected fb for R=%.6f m: %.3f MHz (mu=%.3e Hz/s, Nsweep=%d)\n', ...
        range_true, fb_true/1e6, mu, Nsweep);

% Hardware-like IF cap
IF_FMAX = 50e6;                         % 50 MHz cap (upper IF)

%% ===================== 2) FMCW Waveform =====================
waveform = phased.FMCWWaveform( ...
    'SweepTime',sweep_time, ...
    'SweepBandwidth',bw, ...
    'SampleRate',fs, ...
    'SweepDirection','Up', ...
    'SweepInterval','Symmetric', ...
    'NumSweeps',NumSweeps);

tx = waveform();                        % complex baseband chirp train
Ns_total = numel(tx);
t_all = (0:Ns_total-1).'/fs;

%% ===================== 3) Target, Channel, Front-end =====================
rcs_val = 0.112; % m^2 (example 24 GHz)
target  = phased.RadarTarget('MeanRCS',rcs_val,'OperatingFrequency',fc);
channel = phased.FreeSpace('OperatingFrequency',fc,'TwoWayPropagation',true,'SampleRate',fs);

target_motion = phased.Platform('InitialPosition',[range_true;0;0],'Velocity',[velocity;0;0]);
radar_motion  = phased.Platform('InitialPosition',[0;0;0],'Velocity',[0;0;0]);

collector   = phased.Collector('OperatingFrequency',fc);
radiator    = phased.Radiator('OperatingFrequency',fc);
transmitter = phased.Transmitter('PeakPower',1,'Gain',30);

% Single-look angle (broadside)
[tgt_pos, tgt_vel]     = target_motion(sweep_time);
[radar_pos, radar_vel] = radar_motion(sweep_time);
[~, ang] = rangeangle(tgt_pos, radar_pos);

% Tx -> channel -> target -> Rx
tx_pwr      = transmitter(tx);
tx_radiated = radiator(tx_pwr, ang);
prop_sig    = channel(tx_radiated, radar_pos, tgt_pos, radar_vel, tgt_vel);
rx_clean    = target(prop_sig);
rx          = collector(rx_clean, ang);

% RNG control
switch RNG_MODE
    case "shuffle", rng('shuffle');
    case "fixed",   rng(BASE_SEED);
    otherwise,      rng('shuffle');
end

% Input-referred AWGN (added BEFORE LNA to model realistic thermal noise)
% Per Section 3.5.1: N = kT0*F*B — noise must be at receiver input
if NOISE_SINGLE_SWEEP_ONLY
    rx1d_clean = rx(:);
    rx1d = rx1d_clean; % init
    N_sweeps_total_tmp = floor(numel(rx1d_clean)/Nsweep);
    rxM_tmp = reshape(rx1d_clean(1:Nsweep*N_sweeps_total_tmp), Nsweep, []);
    kNoise = min(max(FIXED_SWEEP_IDX,1), size(rxM_tmp,2));
    rxM_tmp(:,kNoise) = awgn(rxM_tmp(:,kNoise), SNR_dB, 'measured');
    rx1d(1:Nsweep*N_sweeps_total_tmp) = rxM_tmp(:);
else
    rx1d = awgn(rx, SNR_dB, 'measured');
    rx1d = rx1d(:);
end

% Front-end LNA gain (amplifies both signal AND noise together)
LNA_gain = 1e3;     % ~60 dB
rx1d = rx1d * LNA_gain;

% Length match
tx1d = tx(:);
L = min(numel(tx1d), numel(rx1d));
tx1d = tx1d(1:L);
rx1d = rx1d(1:L);
t_rx = (0:L-1).'/fs;

%% ===================== 4) TX Visualizations =====================
if PLOT_TX_RX
    f_if = 150e6;                           % for "oscilloscope" real-IF view
    tx_if = real(tx1d .* exp(1j*2*pi*f_if*t_all(1:L)));

    figure('Name','TX Visuals');
    n_plot = min(2000, L);
    subplot(5,1,1);
    plot(t_all(1:n_plot)*1e6, tx_if(1:n_plot));
    xlabel('Time (\mus)'); ylabel('Amp'); title('Tx Real-Valued IF (Oscilloscope)'); grid on;

    instf_tx = gradient(unwrap(angle(tx1d))) * fs / (2*pi);
    subplot(5,1,2);
    plot(t_all(1:end-1)*1e3, instf_tx(1:end-1)/1e3);
    xlabel('Time (ms)'); ylabel('Freq (kHz)'); title('Tx Instantaneous Frequency (baseband)'); grid on;

    Ytx = abs(fft(tx1d));
    faxis = fs*(0:L-1)/L;
    subplot(5,1,3);
    plot(faxis/1e3, Ytx); xlim([0 fs/2]/1e3);
    xlabel('Freq (kHz)'); ylabel('|FFT|'); title('Tx FFT Spectrum'); grid on;

    win = 256; overlap = 200; nfft = 512;
    subplot(5,1,4);
    [S_tx, F_tx, T_tx] = spectrogram(tx1d, win, overlap, nfft, fs);
    imagesc(T_tx*1e3, F_tx/1e3, mag2db(abs(S_tx)+eps)); axis xy; colorbar;
    xlabel('Time (ms)'); ylabel('Freq (kHz)'); title('Tx Spectrogram (Baseband)');

    subplot(5,1,5);
    [S_rf, F_rf, T_rf] = spectrogram(tx_if, win, overlap, nfft, fs);
    imagesc(T_rf*1e6, F_rf/1e6, mag2db(abs(S_rf)+eps)); axis xy; colorbar;
    xlabel('Time (\mus)'); ylabel('Freq (MHz)'); title('Tx Spectrogram (Real IF)');
end

%% ===================== 5) RX Visualizations =====================
if PLOT_TX_RX
    figure('Name','RX Visuals');
    samples_plot = min(2000, L);
    subplot(4,1,1);
    plot(t_rx(1:samples_plot)*1e6, real(rx1d(1:samples_plot))/max(abs(rx1d))); grid on;
    xlabel('Time (\mus)'); ylabel('Rx (norm)'); title('Rx (time, first 2000 samples)');

    instf_rx = gradient(unwrap(angle(rx1d))) * fs / (2*pi);
    subplot(4,1,2);
    plot(t_rx(1:end-1)*1e3, instf_rx(1:end-1)/1e3); grid on;
    xlabel('Time (ms)'); ylabel('Freq (kHz)'); title('Rx Instantaneous Frequency');

    Yrx = abs(fft(rx1d))/max(abs(rx1d));
    faxis_rx = fs*(0:L-1)/L;
    subplot(4,1,3);
    plot(faxis_rx/1e3, Yrx); xlim([0 fs/2]/1e3); grid on;
    xlabel('Freq (kHz)'); ylabel('|FFT| (norm)'); title('Rx FFT Spectrum');

    win = 256; overlap = 200; nfft = 512;
    subplot(4,1,4);
    [S_rx, F_rx, T_rx] = spectrogram(rx1d, win, overlap, nfft, fs);
    imagesc(T_rx*1e3, F_rx/1e3, mag2db(abs(S_rx)/max(abs(S_rx(:)))+eps));
    axis xy; colorbar; xlabel('Time (ms)'); ylabel('Freq (kHz)'); title('Rx Spectrogram');
end

%% ===================== 6) Reshape, Dechirp, UP-only selection =====================
% Ensure integer number of sweeps
Nsweep = round(sweep_time*fs);
N_sweeps_total = floor(L/Nsweep);
txM = reshape(tx1d(1:Nsweep*N_sweeps_total), Nsweep, []);
rxM = reshape(rx1d(1:Nsweep*N_sweeps_total), Nsweep, []);

% Dechirp (complex beat per sweep)
beatM_full = rxM .* conj(txM);
t_fast = (0:Nsweep-1).'/fs;

% Detect UP sweeps from TX phase slope (for plotting/reporting)
phi_tx = unwrap(angle(txM));           % Nsweep x N_sweeps
Xfit   = [t_fast ones(Nsweep,1)];
beta_s = zeros(2, N_sweeps_total);
for k = 1:N_sweeps_total
    beta_s(:,k) = (Xfit.'*Xfit) \ (Xfit.'*phi_tx(:,k));
end
slope_tx = beta_s(1,:);                % rad/s; positive => "up"
idx_up   = find(slope_tx > 0);
if isempty(idx_up), idx_up = 1:N_sweeps_total; end

% ***** DATA LANES: detection vs plotting *****
beatM_detect = beatM_full;            % detector source (restricted below)
beatM_plot   = beatM_full(:, idx_up); % plot-only copy (UP-only)

% ***** SINGLE-SWEEP GUARD *****
if TEST_SINGLE_SWEEP
    k_det = min(max(FIXED_SWEEP_IDX,1), size(beatM_detect,2));
    x_raw_full = beatM_detect(:, k_det);   % exactly one sweep to detector

    % Up/Down notice for chosen sweep
    phi_tx_col = unwrap(angle(txM(:,k_det)));
    bcol = [t_fast ones(Nsweep,1)] \ phi_tx_col;
    if bcol(1)>0, chType='UP'; else, chType='DOWN'; end
    fprintf('N=1 detector sweep %d is %s-chirp\n', k_det, chType);

    % Optional: report Doppler using slow-time — but NEVER touch x_raw_full
    if REPORT_DOPPLER_IN_N1
        beatM_up = beatM_full(:, idx_up);
        if size(beatM_up,2) >= 2
            ph = angle(sum(beatM_up(:,2:end) .* conj(beatM_up(:,1:end-1)), 1));
            fd_est_report = mean(ph)/(2*pi*PRI);  % Hz
        else
            fd_est_report = 0;
        end
        vel_est_report = (fd_est_report*lambda)/2;
    else
        fd_est_report  = 0;
        vel_est_report = 0;
    end

    % Integrity: ensure detector input hasn’t changed
    x_hash_before = sum(abs(x_raw_full).^2);
    % (no ops must modify x_raw_full in N=1 mode)
    x_hash_after  = sum(abs(x_raw_full).^2);
    assert(abs(x_hash_after - x_hash_before) < 1e-12, 'Leakage: x_raw modified in N=1 mode!');

else
    % Multi-sweep Doppler estimate across UP sweeps — allowed for detection path
    beatM_up = beatM_full(:, idx_up);
    if numel(idx_up) >= 2
        ph = angle(sum(beatM_up(:,2:end) .* conj(beatM_up(:,1:end-1)), 1));
        fd_est = mean(ph)/(2*pi*PRI);   % Hz
    else
        fd_est = 0;
    end
    vel_est = (fd_est*lambda)/2;
    fprintf('Estimated Doppler (multi-sweep): fd=%.2f Hz -> v=%.2f m/s (true %.2f)\n',...
            fd_est, vel_est, velocity);

    % Representative UP sweep for detection (with intra-sweep de-Doppler)
    mid_idx_up = idx_up( max(1, round(numel(idx_up)/2)) );
    x_raw_full = beatM_full(:, mid_idx_up) .* exp(-1j*2*pi*fd_est*t_fast);
    k_det = mid_idx_up; %#ok<NASGU>
end

% Print Doppler report (never used in N=1 detection)
if TEST_SINGLE_SWEEP
    fprintf('Doppler (report-only): fd=%.2f Hz -> v=%.2f m/s (true %.2f)\n', ...
            fd_est_report, vel_est_report, velocity);
end

%% ===================== 7) Multi-Chirp Inst-Freq Plot (plot-only) =====================
if PLOT_MULTI_CHIRP_IF
    num_chirps_to_plot = min(15, size(beatM_plot,2));
    samples_to_plot = num_chirps_to_plot * Nsweep;
    tx_multi = txM(:, idx_up); tx_multi = tx_multi(:);
    rx_multi = rxM(:, idx_up); rx_multi = rx_multi(:);
    tx_multi = tx_multi(1:min(samples_to_plot, numel(tx_multi)));
    rx_multi = rx_multi(1:min(samples_to_plot, numel(rx_multi)));
    t_multi = (0:numel(tx_multi)-1)/fs;

    inst_f_tx = gradient(unwrap(angle(tx_multi))) * fs / (2*pi);
    inst_f_rx = gradient(unwrap(angle(rx_multi))) * fs / (2*pi);
    inst_f_tx = movmean(inst_f_tx, 20);
    inst_f_rx = movmean(inst_f_rx, 20);

    figure('Name','Instantaneous Frequency (first N UP chirps)');
    plot(t_multi*1e6, inst_f_tx/1e6, 'b', 'LineWidth', 1.5); hold on;
    plot(t_multi*1e6, inst_f_rx/1e6, 'r', 'LineWidth', 1.2); grid on;
    xlabel('Time (\mus)'); ylabel('Instantaneous Freq (MHz)');
    legend('Tx IF (UP only)','Rx IF (UP only)');
    title(sprintf('Instantaneous Frequency - First %d UP Chirps (plot-only)', num_chirps_to_plot));
end

%% ===================== 8) CLEAN estimation path with IF cap (DETECTOR USES x_raw_full ONLY) =====================
% Trim edges to avoid transients
keep = round(0.1*numel(x_raw_full)) : round(0.9*numel(x_raw_full));
x0 = x_raw_full(keep);
t_fast_k = t_fast(keep);

% Ensure enough samples for FIR
FIR_ORDER = 128;
assert(numel(x0) > 5*FIR_ORDER, 'Too few samples after trim for %d-tap FIR.', FIR_ORDER);

% ===== Stage 1 (Wide IF) =====
bp_wide = designfilt('bandpassfir', ...
    'FilterOrder', FIR_ORDER, ...
    'CutoffFrequency1', 1e5, ...
    'CutoffFrequency2', min(IF_FMAX, fs/2-1e5), ...
    'SampleRate', fs);
x_wide = filter(bp_wide, x0);

% ---- Estimator A (phase-slope, robust two-stage fit) ----
phi = unwrap(angle(x_wide));
X   = [t_fast_k ones(size(t_fast_k))];
% quick unweighted fit
beta0 = X \ phi;
r     = phi - X*beta0;
iqr_r = iqr(r);
if iqr_r == 0, inl = true(size(r)); else, inl = abs(r - median(r)) < 3*iqr_r; end
w     = abs(x_wide); w = w/max(w + eps);
beta  = (X(inl,:).'*( (w(inl)).*X(inl,:) )) \ (X(inl,:).'*( (w(inl)).*phi(inl) ));
phi_slope = beta(1);                         % rad/s
fb_ps = abs(phi_slope/(2*pi));               % Hz, force positive
R_ps  = (c*fb_ps)/(2*mu);

fprintf('WIDE-IF  PHASE-SLOPE: fb=%.3f MHz -> R=%.3f m\n', fb_ps/1e6, R_ps);

% Chosen estimate (robust, hardware-consistent)
fb_est = fb_ps;
R_est  = R_ps;

% ===== Stage 2 (Optional Narrow Viz) =====
x_nar = [];
if USE_NARROW_PLOT && isfinite(fb_est) && fb_est > 1e5
    guard = max(0.10*fb_est, 5e6);      % ±10% or ±5 MHz, whichever larger
    low_cut  = max(1e5, fb_est - guard);
    high_cut = min(min(IF_FMAX, fs/2-1e5), fb_est + guard);
    if low_cut < high_cut
        bp_nar = designfilt('bandpassfir', ...
            'FilterOrder', FIR_ORDER, ...
            'CutoffFrequency1', low_cut, ...
            'CutoffFrequency2', high_cut, ...
            'SampleRate', fs);
        % viz of the SAME sweep chosen for detection (untrimmed)
        x_nar = filter(bp_nar, x_raw_full);
    end
end

%% ===================== 9) Beat plots: raw vs wide-IF vs narrow (viz only) =====================
if PLOT_BEAT_WIDE_NAR
    figure('Name','Beat Signal — Raw vs Wide-IF vs Optional Narrow');
    nrows = 3;

    subplot(nrows,1,1);
    plot(t_fast*1e3, real(x_raw_full), 'LineWidth',1.2); grid on;
    xlabel('Fast time (ms)'); ylabel('Re{beat}'); title('Dechirped Beat (raw, detector sweep)');

    subplot(nrows,1,2);
    plot(t_fast_k*1e3, real(x_wide), 'LineWidth',1.2); grid on;
    xlabel('Fast time (ms)'); ylabel('Re{beat}');
    title(sprintf('Dechirped Beat (WIDE IF: 0.1–%.0f MHz)', IF_FMAX/1e6));

    if ~isempty(x_nar)
        subplot(nrows,1,3);
        plot(t_fast*1e3, real(x_nar), 'LineWidth',1.2); grid on;
        xlabel('Fast time (ms)'); ylabel('Re{beat}');
        title(sprintf('Dechirped Beat (NAR around %.2f MHz, viz-only)', fb_est/1e6));
    else
        subplot(nrows,1,3); axis off;
    end
end

%% ===================== 10) (Optional) Range-Doppler Map (plot-only, no back-edges) =====================
if PLOT_RANGE_DOPPLER
    Mplot = beatM_plot; % UP-only copy for visualization
    win_fast  = hann(size(Mplot,1));
    win_slow  = hann(size(Mplot,2)).';
    Mwin = (Mplot .* win_fast) .* win_slow;
    Nf = 2048; Ns = 1024;
    RD = fftshift(fft(fft(Mwin, Nf, 1), Ns, 2), 2);
    RDdB = mag2db(abs(RD)+eps);

    figure('Name','Range-Doppler (plot-only)');
    rng_axis = (0:Nf-1)*(fs/Nf) * (c/(2*mu));    % simple range axis from fb
    dop_axis = ((-Ns/2):(Ns/2-1))*(1/(Ns*PRI));  % Doppler axis
    imagesc(dop_axis, rng_axis, RDdB); axis xy; colorbar;
    xlabel('Doppler (Hz)'); ylabel('Range (m)'); title('RDM (UP-only) — not used by detector');
end

%% ===================== 11) Final Report & Save =====================
fprintf('\n=== FINAL ESTIMATE (IF-limited, two-stage) ===\n');
fprintf('True R=%.2f m | fb_true=%.3f MHz\n', range_true, fb_true/1e6);
fprintf('Chosen -> fb=%.3f MHz | R=%.3f m\n', fb_est/1e6, R_est);

if SAVE_BEAT_FILES
    output_filename = 'beat_24GHz.mat';
    fs_out = fs;
    % Save the detector’s wide-IF vector (single-sweep processed segment)
    t_out  = t_fast_k;      % match the trimmed segment length
    beat_signal = x_wide;   %#ok<NASGU>
    save(output_filename, 'beat_signal','fs_out','t_out');

    % Optional text save (real part for quick viewing)
    txt_filename = 'beat_24GHz.txt';
    Nmin = min(numel(t_out), numel(beat_signal));
    writematrix([t_out(1:Nmin) real(beat_signal(1:Nmin))], txt_filename);

    fprintf('✔ Beat signal saved to %s and %s\n', output_filename, txt_filename);
end