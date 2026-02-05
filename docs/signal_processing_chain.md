# FMCW Radar Signal Processing Chain - Detailed Documentation

**Project**: Multi-Band Radar Fusion Research  
**Date**: February 4, 2026  
**Models**: R1_model.m (5.8 GHz), R2_model.m (24 GHz)

---

## Table of Contents
1. [High-Level System Overview](#high-level-system-overview)
2. [Detailed Processing Stages](#detailed-processing-stages)
3. [Data Flow Diagrams](#data-flow-diagrams)
4. [Mathematical Formulations](#mathematical-formulations)
5. [Signal Characteristics at Each Stage](#signal-characteristics-at-each-stage)
6. [Critical Design Decisions](#critical-design-decisions)

---

## High-Level System Overview

### Complete Signal Processing Pipeline

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                    FMCW RADAR SIGNAL PROCESSING CHAIN                        │
└──────────────────────────────────────────────────────────────────────────────┘

                            ┌─────────────┐
                            │ Section 0   │
                            │  TOGGLES    │
                            │ & CONFIG    │
                            └──────┬──────┘
                                   │
                            ┌──────▼──────┐
                            │ Section 1   │
                            │ CONSTANTS   │
                            │  & SPECS    │
                            └──────┬──────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │      Section 2              │
                    │   FMCW WAVEFORM GENERATION  │
                    │   (phased.FMCWWaveform)     │
                    │                             │
                    │ Output: tx [N×1] complex    │
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │      Section 3              │
                    │   TARGET MODEL              │
                    │   (Swerling II / Static)    │
                    │   + CHANNEL PROPAGATION     │
                    │                             │
                    │ Output: rx [N×1] complex    │
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │    Section 3a               │
                    │  RF FRONT-END PROCESSING    │
                    │  1. Add AWGN (input-ref)    │
                    │  2. Apply LNA gain (60 dB)  │
                    │                             │
                    │ Output: rx1d [N×1] complex  │
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │   Sections 4-5              │
                    │   TX/RX VISUALIZATION       │
                    │   (Time, Freq, Spectrogram) │
                    │                             │
                    │ [Plot-only, no data change] │
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │      Section 6              │
                    │   DECHIRP & SWERLING        │
                    │   1. Reshape to matrix      │
                    │   2. Conjugate multiply     │
                    │   3. Apply RCS fluctuation  │
                    │   4. UP-sweep detection     │
                    │                             │
                    │ Output: beatM_full [Ns×M]   │
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │      Section 7              │
                    │  MULTI-CHIRP INST-FREQ      │
                    │  (Visualization only)       │
                    │                             │
                    │ [Plot-only, no data change] │
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │      Section 8              │
                    │   RANGE ESTIMATION          │
                    │   1. Select sweep (N=1/mid) │
                    │   2. Edge trimming          │
                    │   3. Wide-IF bandpass       │
                    │   4. Phase-slope fitting    │
                    │                             │
                    │ Output: R_est, fb_est       │
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │     Section 8a              │
                    │   AFS PROCESSING            │
                    │   (Adaptive Fusion Select)  │
                    │   1. Range-cell detection   │
                    │   2. CFAR gating            │
                    │   3. Nullify low-SNR pulses │
                    │                             │
                    │ Output: beat_cleaned [Ns×M']│
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │   Sections 9-10             │
                    │   VISUALIZATION             │
                    │   (Beat plots, RDM)         │
                    │                             │
                    │ [Plot-only, no data change] │
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │     Section 11              │
                    │   SAVE & REPORT             │
                    │   (MAT/TXT output)          │
                    │                             │
                    │ Output: beat_5_8GHz.mat     │
                    └─────────────────────────────┘
```

---

## Detailed Processing Stages

### Stage 0: Configuration & Toggles

**Purpose**: Central control for all processing modes and visualization flags

```
┌──────────────────────────────────────────────────────┐
│              CONFIGURATION PARAMETERS                │
├──────────────────────────────────────────────────────┤
│                                                      │
│  ┌────────────────────────────────────────────┐     │
│  │ VISUALIZATION TOGGLES                      │     │
│  │  • PLOT_TX_RX            (true/false)      │     │
│  │  • PLOT_BEAT_WIDE_NAR    (true/false)      │     │
│  │  • PLOT_MULTI_CHIRP_IF   (true/false)      │     │
│  │  • PLOT_RANGE_DOPPLER    (true/false)      │     │
│  │  • USE_NARROW_PLOT       (true/false)      │     │
│  │  • PLOT_SWERLING         (true/false)      │     │
│  │  • PLOT_AFS_PROCESSING   (true/false)      │     │
│  └────────────────────────────────────────────┘     │
│                                                      │
│  ┌────────────────────────────────────────────┐     │
│  │ PROCESSING MODES                           │     │
│  │  • APPLY_AFS             (true/false)      │     │
│  │  • USE_SWERLING_II       (true/false)      │     │
│  │  • TEST_SINGLE_SWEEP     (true/false)      │     │
│  │  • NOISE_SINGLE_SWEEP    (true/false)      │     │
│  │  • SAVE_BEAT_FILES       (true/false)      │     │
│  └────────────────────────────────────────────┘     │
│                                                      │
│  ┌────────────────────────────────────────────┐     │
│  │ RNG CONTROL                                │     │
│  │  • RNG_MODE: "shuffle" / "fixed"           │     │
│  │  • BASE_SEED: 54321 (R1) / 12345 (R2)      │     │
│  └────────────────────────────────────────────┘     │
│                                                      │
└──────────────────────────────────────────────────────┘
```

**Key Design Choice**: All modes controlled from single location for:
- Reproducible research (RNG control)
- Quick scenario testing (toggle visualization)
- Mode integrity (N=1 prevents slow-time leakage)

---

### Stage 1: System Constants & Specifications

**Purpose**: Define radar parameters and scenario

```
┌─────────────────────────────────────────────────────────────┐
│                 SYSTEM SPECIFICATIONS                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  PHYSICAL CONSTANTS:                                        │
│  ┌──────────────────────────────────────────┐              │
│  │ c  = 3×10⁸ m/s (speed of light)          │              │
│  └──────────────────────────────────────────┘              │
│                                                             │
│  RADAR PARAMETERS:                                          │
│  ┌──────────────────────────────────────────┐              │
│  │ fc    = 5.8 GHz  (R1) / 24 GHz  (R2)     │              │
│  │ λ     = c/fc     (wavelength)            │              │
│  │ Rmax  = 100 m    (max unambiguous range) │              │
│  │ bw    = 150 MHz  (sweep bandwidth)       │              │
│  │ fs    = 300 MHz  (sampling rate = 2×bw)  │              │
│  │ sweep = 3.33 μs  (chirp duration)        │              │
│  │ M     = 512      (number of sweeps)      │              │
│  └──────────────────────────────────────────┘              │
│                                                             │
│  DERIVED PARAMETERS:                                        │
│  ┌──────────────────────────────────────────┐              │
│  │ μ      = bw/sweep_time (Hz/s)            │              │
│  │ Nsweep = round(sweep_time × fs)          │              │
│  │ PRI    = sweep_time (no idle)            │              │
│  │ fb_true= μ × (2R/c) (expected beat freq) │              │
│  └──────────────────────────────────────────┘              │
│                                                             │
│  SCENARIO:                                                  │
│  ┌──────────────────────────────────────────┐              │
│  │ range_true = 37 m    (target distance)   │              │
│  │ velocity   = 50 m/s  (radial velocity)   │              │
│  │ SNR_dB     = 0.11 dB (R1) / -4 dB (R2)   │              │
│  └──────────────────────────────────────────┘              │
│                                                             │
│  HARDWARE CONSTRAINTS:                                      │
│  ┌──────────────────────────────────────────┐              │
│  │ IF_FMAX = 50 MHz (IF bandpass limit)     │              │
│  │ LNA_gain = 60 dB (front-end amplifier)   │              │
│  │ TX_gain  = 30 dB (transmitter gain)      │              │
│  └──────────────────────────────────────────┘              │
│                                                             │
└─────────────────────────────────────────────────────────────┘

OUTPUT:
• All parameters available to downstream stages
• Expected beat frequency calculated: fb_true ≈ 3.7 MHz
• Console printout for verification
```

---

### Stage 2: FMCW Waveform Generation

**Purpose**: Create frequency-modulated continuous wave chirp train

```
                    WAVEFORM GENERATION
                  (phased.FMCWWaveform)

Input Parameters:          Output Signal:
┌─────────────────┐       ┌──────────────────────┐
│ SweepTime       │       │                      │
│ SweepBandwidth  │       │  tx [Ns_total × 1]   │
│ SampleRate      │       │                      │
│ SweepDirection  │──────▶│  Complex baseband    │
│ SweepInterval   │       │  chirp train         │
│ NumSweeps       │       │                      │
└─────────────────┘       │  Ns_total = M × Ns   │
                          └──────────────────────┘

Time-Domain Representation:
┌────────────────────────────────────────────────────┐
│ tx(t) = exp(j·π·μ·t²)  for 0 ≤ t ≤ sweep_time    │
│                                                    │
│ where: μ = bw/sweep_time (chirp rate, Hz/s)      │
└────────────────────────────────────────────────────┘

Frequency Sweep Pattern:
    │
    │     ╱│     ╱│     ╱│     ╱│
  f │    ╱ │    ╱ │    ╱ │    ╱ │
  c │   ╱  │   ╱  │   ╱  │   ╱  │
  + │  ╱   │  ╱   │  ╱   │  ╱   │
  B │ ╱    │ ╱    │ ╱    │ ╱    │
  W │╱     │╱     │╱     │╱     │
    ├──────┼──────┼──────┼──────┼───▶ time
      Tc     Tc     Tc     Tc
    [Chirp 1][Chirp 2][Chirp 3][Chirp M]

Signal Properties:
• Length: Ns_total = NumSweeps × Nsweep samples
• Bandwidth: 150 MHz (instantaneous)
• Duration: ~1.7 ms total (512 sweeps × 3.33 μs)
• Type: Complex baseband (I/Q)
• Phase: Quadratic (Φ(t) = π·μ·t²)
```

**Mathematical Details**:

The transmitted signal is:
$$s_{tx}(t) = \exp\left(j\pi\mu t^2\right), \quad 0 \leq t \leq T_c$$

where:
- $\mu = B/T_c$ is the chirp rate (Hz/s)
- $B = 150$ MHz is the sweep bandwidth
- $T_c \approx 3.33$ μs is the chirp duration

Instantaneous frequency:
$$f_{inst}(t) = \frac{1}{2\pi}\frac{d\phi}{dt} = \mu t$$

---

### Stage 3: Target Model & Channel Propagation

**Purpose**: Simulate radar cross-section (RCS) fluctuation and signal propagation

```
┌────────────────────────────────────────────────────────────────┐
│                    TARGET MODELING PIPELINE                    │
└────────────────────────────────────────────────────────────────┘

Step 1: Swerling II RCS Model (if USE_SWERLING_II = true)
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  SwerlingIITarget Class:                                 │
│  ┌────────────────────────────────────────────────┐     │
│  │ Input:                                         │     │
│  │  • Mean RCS (dBsm): -17 (R1) / -9.5 (R2)      │     │
│  │  • NumSweeps: M = 512                          │     │
│  │  • Operating frequency: fc                     │     │
│  │  • RNG mode: shuffle/fixed                     │     │
│  └────────────────────────────────────────────────┘     │
│                                                          │
│  Processing:                                             │
│  ┌────────────────────────────────────────────────┐     │
│  │ 1. Generate IID complex amplitudes α[m]        │     │
│  │    where |α[m]|² ~ Chi-squared(2 DOF)          │     │
│  │                                                 │     │
│  │ 2. RCS[m] = |α[m]|² × RCS_mean                 │     │
│  │                                                 │     │
│  │ 3. Normalize: ᾱ[m] = α[m]/√E[|α|²]            │     │
│  │    (unit mean power, preserves fluctuation)    │     │
│  └────────────────────────────────────────────────┘     │
│                                                          │
│  Output:                                                 │
│  ┌────────────────────────────────────────────────┐     │
│  │ • alpha_normalized [M × 1]: Complex gains      │     │
│  │ • rcs_fluctuating [M × 1]: RCS per chirp (m²)  │     │
│  │ • afs_data: Structure for downstream AFS       │     │
│  └────────────────────────────────────────────────┘     │
│                                                          │
└──────────────────────────────────────────────────────────┘

RCS Fluctuation Characteristics:
┌─────────────────────────────────────────────────────────┐
│  Swerling II Statistics:                                │
│  • Distribution: Exponential (linear power)             │
│  • Mean: RCS_mean (constant across chirps)              │
│  • Variance: RCS_mean² (100% relative std)              │
│  • Fluctuation range: ~20 dB (10×log₁₀(100))            │
│  • Decorrelation: Pulse-to-pulse (IID)                  │
│                                                          │
│  Physical Meaning:                                       │
│  • Models UAV in resonance region                       │
│  • Multiple scatterers with random phases               │
│  • Justifies need for AFS (adaptive fusion)             │
└─────────────────────────────────────────────────────────┘

Step 2: Channel Propagation (phased.FreeSpace)
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  tx_pwr ──▶ tx_radiated ──▶ prop_sig ──▶ rx_clean ──▶ rx│
│             (radiator)      (channel)    (target)   (coll)│
│                                                          │
│  Signal Path:                                            │
│  ┌────────────────────────────────────────────────┐     │
│  │ 1. Transmitter gain: +30 dB                    │     │
│  │ 2. Free-space path loss: -FSPL(R, fc)          │     │
│  │ 3. Target reflection: RCS_mean (static)        │     │
│  │ 4. Return path loss: -FSPL(R, fc)              │     │
│  │ 5. Collector (receive antenna)                 │     │
│  └────────────────────────────────────────────────┘     │
│                                                          │
│  Free-Space Path Loss (two-way):                        │
│  FSPL = (4πR/λ)⁴                                        │
│                                                          │
│  Received Power (Radar Range Equation):                 │
│  Pr = Pt · G² · λ² · σ / [(4π)³ · R⁴]                   │
│                                                          │
└──────────────────────────────────────────────────────────┘

OUTPUT:
┌──────────────────────────────────────────────┐
│ rx [Ns_total × 1]                            │
│                                              │
│ Clean received signal (pre-noise, pre-LNA)   │
│ Contains:                                     │
│  • Target echo (time-delayed)                │
│  • Doppler shift (if velocity ≠ 0)          │
│  • Path loss attenuation                     │
│  • NO RCS fluctuation yet (applied in Sec 6) │
└──────────────────────────────────────────────┘
```

**Key Design Decision**: 
- Use **static** `phased.RadarTarget` during propagation
- Apply Swerling II amplitude scaling **after dechirp** (Section 6)
- This approach matches MathWorks best practices for fluctuating targets

---

### Stage 3a: RF Front-End Processing

**Purpose**: Add input-referred noise and apply LNA gain (corrected noise model)

```
┌──────────────────────────────────────────────────────────────┐
│             RF FRONT-END SIGNAL CONDITIONING                 │
│          (CRITICAL: INPUT-REFERRED NOISE MODEL)              │
└──────────────────────────────────────────────────────────────┘

                        ┌──────────────┐
                        │   rx (clean) │
                        │  [Ns_total×1]│
                        └──────┬───────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │ STEP 1: RNG Control  │
                    │  • shuffle / fixed   │
                    │  • Set random seed   │
                    └──────────┬───────────┘
                               │
                               ▼
         ┌─────────────────────────────────────────┐
         │ STEP 2: Input-Referred AWGN             │
         │                                         │
         │  Mode A: Normal (all sweeps)            │
         │  ┌───────────────────────────────┐     │
         │  │ rx1d = awgn(rx, SNR_dB)       │     │
         │  └───────────────────────────────┘     │
         │                                         │
         │  Mode B: Single-Sweep Noise             │
         │  ┌───────────────────────────────┐     │
         │  │ 1. Reshape to matrix [Ns×M]   │     │
         │  │ 2. Add noise to sweep k only  │     │
         │  │ 3. Reshape back to vector     │     │
         │  └───────────────────────────────┘     │
         │                                         │
         │  Thermal Noise Model:                   │
         │  N = kT₀FB                              │
         │  where:                                 │
         │   k  = Boltzmann constant               │
         │   T₀ = 290 K (room temperature)         │
         │   F  = Noise figure (implicit in SNR)   │
         │   B  = Bandwidth (150 MHz)              │
         └─────────────────┬───────────────────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │ rx1d [Ns_total × 1]  │
                │ (noisy, pre-LNA)     │
                └──────────┬───────────┘
                           │
                           ▼
         ┌─────────────────────────────────────────┐
         │ STEP 3: Front-End LNA Amplification     │
         │                                         │
         │  ┌───────────────────────────────┐     │
         │  │ rx1d = rx1d × LNA_gain        │     │
         │  │                               │     │
         │  │ LNA_gain = 1000 (60 dB)       │     │
         │  └───────────────────────────────┘     │
         │                                         │
         │  CRITICAL: LNA amplifies BOTH           │
         │  signal AND noise together              │
         │  (realistic RF chain behavior)          │
         └─────────────────┬───────────────────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │ rx1d [Ns_total × 1]  │
                │ (noisy, post-LNA)    │
                │                      │
                │ SNR maintained       │
                │ (both signal & noise │
                │  amplified equally)  │
                └──────────┬───────────┘
                           │
                           ▼
                    Length Matching
                ┌──────────────────────┐
                │ L = min(|tx|, |rx|)  │
                │ tx1d = tx(1:L)       │
                │ rx1d = rx(1:L)       │
                │ t_rx = (0:L-1)/fs    │
                └──────────────────────┘

OUTPUT:
┌────────────────────────────────────────────────────────┐
│ tx1d [L × 1]: Transmitted chirp train (clean)         │
│ rx1d [L × 1]: Received echo (noisy, post-LNA)         │
│ t_rx [L × 1]: Time vector (seconds)                   │
│                                                        │
│ Ready for dechirp processing                           │
└────────────────────────────────────────────────────────┘
```

**Physics Validation**:

Before fix (INCORRECT):
```
rx → LNA(×1000) → Add noise → rx1d
     ↑                ↑
   Signal      Noise added to
 amplified    amplified signal
              (over-optimistic!)
```

After fix (CORRECT):
```
rx → Add noise → LNA(×1000) → rx1d
     ↑               ↑
  Thermal       Both signal+noise
  noise at    amplified together
  input        (physically accurate)
```

**SNR Preservation**:
$$\text{SNR}_{\text{out}} = \frac{P_s \cdot G^2}{N_0 \cdot G^2} = \frac{P_s}{N_0} = \text{SNR}_{\text{in}}$$

where $G = \text{LNA\_gain} = 1000$ (60 dB).

---

### Stage 4-5: TX/RX Visualization (Plot-Only)

**Purpose**: Diagnostic visualization of transmitted and received signals

```
┌──────────────────────────────────────────────────────┐
│         TX/RX VISUALIZATION (Plot-Only)              │
│         No modification to signal data               │
└──────────────────────────────────────────────────────┘

TX Visualizations (Section 4):
┌────────────────────────────────────────────────────┐
│ 1. Real-Valued IF (Oscilloscope View)             │
│    tx_if = Re{tx1d · exp(j2πf_IF·t)}              │
│    Shows: Amplitude-modulated carrier             │
│                                                    │
│ 2. Instantaneous Frequency (Baseband)             │
│    f_inst = gradient(unwrap(∠tx1d)) × fs/(2π)     │
│    Shows: Linear FM sweep (ramp)                  │
│                                                    │
│ 3. FFT Spectrum                                    │
│    Shows: Wideband energy distribution            │
│                                                    │
│ 4. Spectrogram (Baseband)                         │
│    Shows: Time-frequency evolution (chirps)       │
│                                                    │
│ 5. Spectrogram (Real IF)                          │
│    Shows: RF-like representation                  │
└────────────────────────────────────────────────────┘

RX Visualizations (Section 5):
┌────────────────────────────────────────────────────┐
│ 1. Time-Domain Waveform                            │
│    Shows: Echo with noise, first 2000 samples     │
│                                                    │
│ 2. Instantaneous Frequency                         │
│    Shows: Doppler-shifted chirp slope             │
│                                                    │
│ 3. FFT Spectrum (Normalized)                       │
│    Shows: Spectral content with noise floor       │
│                                                    │
│ 4. Spectrogram                                     │
│    Shows: Time-frequency with SNR variation       │
└────────────────────────────────────────────────────┘

Controlled by PLOT_TX_RX toggle
```

---

### Stage 6: Dechirp & Swerling II Application

**Purpose**: Extract beat frequency and apply RCS fluctuation

```
┌──────────────────────────────────────────────────────────────┐
│                  DECHIRP & FLUCTUATION STAGE                 │
└──────────────────────────────────────────────────────────────┘

STEP 1: Reshape to Fast-Time / Slow-Time Matrix
┌────────────────────────────────────────────────────┐
│  Vector to Matrix Conversion:                     │
│                                                    │
│  tx1d [L×1] ────▶ txM [Nsweep × M]                │
│  rx1d [L×1] ────▶ rxM [Nsweep × M]                │
│                                                    │
│  where:                                            │
│   Nsweep = samples per chirp                      │
│   M = number of chirps                            │
│                                                    │
│  Columns = individual chirps (slow time)          │
│  Rows = fast-time samples within chirp           │
└────────────────────────────────────────────────────┘

STEP 2: Dechirp (Conjugate Multiplication)
┌────────────────────────────────────────────────────┐
│  beatM_full = rxM .* conj(txM)                     │
│                                                    │
│  For each chirp m:                                 │
│  beat[:,m] = rx[:,m] · tx*[:,m]                    │
│                                                    │
│  Frequency mixing:                                 │
│  ┌──────────────────────────────────────┐         │
│  │ TX: exp(jπμt²)                        │         │
│  │ RX: exp(jπμ(t-τ)²) · exp(j2πfₐt)     │         │
│  │                                       │         │
│  │ Beat = RX · TX*                       │         │
│  │      ≈ exp(-j2πfᵦt) · exp(j2πfₐt)    │         │
│  │                                       │         │
│  │ where:                                │         │
│  │  fᵦ = μτ (beat frequency from range) │         │
│  │  fₐ = Doppler shift                   │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Result: beatM_full [Nsweep × M] (complex)        │
└────────────────────────────────────────────────────┘

STEP 3: Apply Swerling II Amplitude Scaling
┌────────────────────────────────────────────────────┐
│  if USE_SWERLING_II == true:                       │
│                                                    │
│  For each chirp m = 1 to M:                        │
│    beatM_full[:,m] ← beatM_full[:,m] × α[m]        │
│                                                    │
│  where α[m] ~ Swerling II distribution             │
│                                                    │
│  ┌──────────────────────────────────────┐         │
│  │ α Generation (SwerlingIITarget):     │         │
│  │                                       │         │
│  │ 1. Sample α_re, α_im ~ N(0,1)        │         │
│  │ 2. α_complex = (α_re + jα_im)/√2     │         │
│  │ 3. |α|² ~ Exp(1) (chi-sq 2 DOF)      │         │
│  │ 4. Normalize: E[|α|²] = 1            │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Measurement Verification:                         │
│  ┌──────────────────────────────────────┐         │
│  │ For each chirp m:                     │         │
│  │   P[m] = mean(|beat[:,m]|²)           │         │
│  │   P_dB[m] = 10·log₁₀(P[m])            │         │
│  │                                       │         │
│  │ Verify: range(P_dB) ≈ 20 dB           │         │
│  │         std(P_dB) ~ 5-6 dB            │         │
│  └──────────────────────────────────────┘         │
└────────────────────────────────────────────────────┘

STEP 4: UP-Sweep Detection
┌────────────────────────────────────────────────────┐
│  Goal: Identify chirps with positive FM slope      │
│  (UP sweeps vs DOWN sweeps)                        │
│                                                    │
│  For each chirp m:                                 │
│  1. Extract TX phase: φ[m] = unwrap(∠txM[:,m])    │
│  2. Fit linear model: φ = β₀ + β₁·t               │
│  3. Slope: s[m] = β₁ (rad/s)                      │
│  4. Classify: UP if s[m] > 0                      │
│                                                    │
│  idx_up = indices where slope > 0                 │
│                                                    │
│  ┌──────────────────────────────────────┐         │
│  │ Why UP-only?                          │         │
│  │ • Avoid triangle-wave ambiguity       │         │
│  │ • Consistent range-Doppler mapping    │         │
│  │ • Simplifies phase processing         │         │
│  └──────────────────────────────────────┘         │
└────────────────────────────────────────────────────┘

STEP 5: Data Lane Separation
┌────────────────────────────────────────────────────┐
│  beatM_detect = beatM_full  (all chirps)           │
│    ↓                                               │
│    Used for DETECTION path (range estimation)     │
│                                                    │
│  beatM_plot = beatM_full(:, idx_up)  (UP-only)     │
│    ↓                                               │
│    Used for PLOTTING (RDM, visualization)         │
│                                                    │
│  CRITICAL: Plotting never feeds back to detector  │
│            (prevents data leakage)                │
└────────────────────────────────────────────────────┘

OUTPUT:
┌────────────────────────────────────────────────────┐
│ beatM_full [Nsweep × M]                            │
│  • Complex beat signal matrix                     │
│  • Contains Swerling II fluctuations              │
│  • Ready for single-sweep or multi-sweep proc     │
│                                                    │
│ chirp_power_dB [M × 1]                             │
│  • Per-chirp power measurement (dB)               │
│  • Shows ~20 dB fluctuation range                 │
│  • Used for AFS candidate selection               │
│                                                    │
│ idx_up [M' × 1]                                    │
│  • Indices of UP-sweeps                           │
│  • M' ≤ M (typically M' ≈ M/2)                    │
└────────────────────────────────────────────────────┘
```

**Mathematical Detail - Beat Frequency**:

Transmitted signal:
$$s_{tx}(t) = \exp(j\pi\mu t^2)$$

Received signal (target at range $R$, velocity $v$):
$$s_{rx}(t) = A\exp\left(j\pi\mu(t-\tau)^2\right)\exp(j2\pi f_d t)$$

where:
- $\tau = 2R/c$ (round-trip delay)
- $f_d = 2v/\lambda$ (Doppler shift)

Beat signal after dechirp:
$$s_{beat}(t) = s_{rx}(t) \cdot s_{tx}^*(t) \approx A\exp(-j2\pi f_b t)\exp(j2\pi f_d t)$$

where beat frequency:
$$f_b = \mu\tau = \frac{2\mu R}{c}$$

---

### Stage 7: Multi-Chirp Instantaneous Frequency (Plot-Only)

**Purpose**: Visualize frequency evolution across multiple chirps

```
Controlled by PLOT_MULTI_CHIRP_IF toggle
Displays first N UP-chirps (typically 15)
No data modification
```

---

### Stage 8: Range Estimation (Phase-Slope Method)

**Purpose**: Extract range estimate from beat signal phase

```
┌──────────────────────────────────────────────────────────────┐
│              RANGE ESTIMATION PIPELINE                       │
└──────────────────────────────────────────────────────────────┘

STEP 1: Sweep Selection
┌────────────────────────────────────────────────────┐
│  Mode A: Single-Sweep (TEST_SINGLE_SWEEP = true)  │
│  ┌──────────────────────────────────────┐         │
│  │ k_det = FIXED_SWEEP_IDX (e.g., 1)    │         │
│  │ x_raw_full = beatM_detect[:, k_det]  │         │
│  │                                       │         │
│  │ INTEGRITY CHECK:                     │         │
│  │  hash_before = Σ|x_raw_full|²        │         │
│  │  [processing...]                     │         │
│  │  hash_after = Σ|x_raw_full|²         │         │
│  │  assert |hash_after-hash_before|<ε   │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Mode B: Multi-Sweep (TEST_SINGLE_SWEEP = false)  │
│  ┌──────────────────────────────────────┐         │
│  │ 1. Select middle UP-sweep             │         │
│  │ 2. Estimate Doppler: fₐ = Δφ/(2πPRI) │         │
│  │ 3. De-Doppler correction:             │         │
│  │    x_raw = beat · exp(-j2πfₐt)        │         │
│  └──────────────────────────────────────┘         │
└────────────────────────────────────────────────────┘

STEP 2: Edge Trimming (Transient Rejection)
┌────────────────────────────────────────────────────┐
│  keep = 0.1×N : 0.9×N  (middle 80%)                │
│  x0 = x_raw_full[keep]                             │
│  t_fast_k = t_fast[keep]                           │
│                                                    │
│  Reason: Avoid filter startup transients          │
└────────────────────────────────────────────────────┘

STEP 3: Wide-IF Bandpass Filter
┌────────────────────────────────────────────────────┐
│  Design: FIR bandpass                              │
│  ┌──────────────────────────────────────┐         │
│  │ Order: 128 taps                       │         │
│  │ f_low: 0.1 MHz (reject DC drift)      │         │
│  │ f_high: 50 MHz (IF_FMAX cap)          │         │
│  │ Sample rate: 300 MHz                  │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  x_wide = filter(bp_wide, x0)                      │
│                                                    │
│  Purpose: Hardware-realistic IF constraint        │
└────────────────────────────────────────────────────┘

STEP 4: Phase-Slope Estimation (Robust Two-Stage)
┌────────────────────────────────────────────────────┐
│  Phase Extraction:                                 │
│  φ = unwrap(∠x_wide)  [radians]                    │
│                                                    │
│  Stage 1: Quick Unweighted Fit                     │
│  ┌──────────────────────────────────────┐         │
│  │ Model: φ = β₀ + β₁·t                  │         │
│  │ X = [t_fast_k, ones(N,1)]             │         │
│  │ β⁰ = (XᵀX)⁻¹Xᵀφ  (least squares)      │         │
│  │                                       │         │
│  │ Residuals: r = φ - Xβ⁰                │         │
│  │ Outlier detection:                    │         │
│  │  inliers = |r - median(r)| < 3·IQR(r)│         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Stage 2: Weighted Fit (Inliers Only)              │
│  ┌──────────────────────────────────────┐         │
│  │ Weights: w = |x_wide| (amplitude)     │         │
│  │ w = w/max(w)  (normalize)             │         │
│  │                                       │         │
│  │ Weighted LS:                          │         │
│  │ β = (XᵀWX)⁻¹XᵀWφ                      │         │
│  │ where W = diag(w[inliers])            │         │
│  │                                       │         │
│  │ Extract slope: β₁ (rad/s)             │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Frequency Conversion:                             │
│  ┌──────────────────────────────────────┐         │
│  │ fᵦ = |β₁|/(2π)  [Hz]                  │         │
│  │                                       │         │
│  │ Range:                                │         │
│  │ R = c·fᵦ/(2μ)  [meters]               │         │
│  │                                       │         │
│  │ where μ = bw/sweep_time               │         │
│  └──────────────────────────────────────┘         │
└────────────────────────────────────────────────────┘

OUTPUT:
┌────────────────────────────────────────────────────┐
│ fb_est: Estimated beat frequency (Hz)              │
│ R_est:  Estimated range (meters)                   │
│                                                    │
│ Accuracy: Limited by:                              │
│  • Phase unwrapping errors                        │
│  • Noise-induced slope variation                  │
│  • IF bandpass edge effects                       │
│                                                    │
│ Typical performance:                               │
│  SNR > 0 dB → error < 0.5 m                       │
│  SNR < -5 dB → error > 2 m                        │
└────────────────────────────────────────────────────┘
```

**Phase-Slope Visualization**:

```
φ (rad) │      ╱
        │     ╱          Ideal: Linear phase
        │    ╱           Slope = 2πfᵦ
        │   ╱
        │  ╱     ×       × = noisy samples
        │ ╱   ×  ×       
        │╱ ×       ×     Outliers rejected
        └────────────────▶ time (s)
        
        With noise:
        • Phase unwrap errors
        • Amplitude weighting helps
        • Robust fit removes outliers
```

---

### Stage 8a: AFS Processing (Adaptive Fusion Selection)

**Purpose**: Suppress low-SNR chirps caused by Swerling II RCS nulls

```
┌──────────────────────────────────────────────────────────────┐
│          AFS: AMPLITUDE FLUCTUATION SUPPRESSION              │
│          (Honest Mode - Uses Estimated Range)                │
└──────────────────────────────────────────────────────────────┘

Triggered when:
• APPLY_AFS = true
• USE_SWERLING_II = true
• Range estimate R_est available

ALGORITHM OVERVIEW:
┌────────────────────────────────────────────────────┐
│  For each chirp m in beatM_full:                   │
│                                                    │
│  1. Extract range-cell around R_est               │
│  2. Compute local SNR using CFAR logic            │
│  3. If SNR < threshold: NULLIFY chirp             │
│  4. Return cleaned beat matrix                    │
└────────────────────────────────────────────────────┘

STEP 1: Range-Cell Selection
┌────────────────────────────────────────────────────┐
│  Range resolution: ΔR = c/(2·bw) ≈ 1 m            │
│                                                    │
│  For estimated range R_est:                        │
│  ┌──────────────────────────────────────┐         │
│  │ Detection cell index:                 │         │
│  │   k_det = round(R_est / ΔR)           │         │
│  │                                       │         │
│  │ Protection cells (Doppler guard):     │         │
│  │   n_guard = ceil(v_max·Tᵣ / ΔR)      │         │
│  │                                       │         │
│  │ Reference cells:                      │         │
│  │   n_ref = 8 per side                  │         │
│  │   (used for noise floor estimation)   │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Cell Layout:                                      │
│  ┌────────────────────────────────────────┐       │
│  │ [REF] [REF] ... [GUARD] [DET] [GUARD]  │       │
│  │  ... [REF] [REF]                        │       │
│  │                                         │       │
│  │  ←───────────────────────────────────→ │       │
│  │     n_ref+n_guard+1+n_guard+n_ref       │       │
│  └────────────────────────────────────────┘       │
└────────────────────────────────────────────────────┘

STEP 2: CFAR Detection Logic
┌────────────────────────────────────────────────────┐
│  For each chirp m:                                 │
│                                                    │
│  FFT to range domain:                              │
│  ┌──────────────────────────────────────┐         │
│  │ Y[m] = FFT(beatM_full[:,m])           │         │
│  │ P[k] = |Y[m,k]|²  (power vs range)    │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Detection cell power:                             │
│  ┌──────────────────────────────────────┐         │
│  │ P_det = P[k_det]                      │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Reference cell average (noise floor):             │
│  ┌──────────────────────────────────────┐         │
│  │ P_ref = mean(P[ref_cells])            │         │
│  │ (excludes detection & guard cells)    │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Local SNR:                                        │
│  ┌──────────────────────────────────────┐         │
│  │ SNR_local[m] = 10·log₁₀(P_det/P_ref)  │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Threshold decision:                               │
│  ┌──────────────────────────────────────┐         │
│  │ threshold_dB = SNR_mean - 6 dB        │         │
│  │                                       │         │
│  │ if SNR_local[m] < threshold_dB:       │         │
│  │    beat_cleaned[:,m] = 0 (NULLIFY)    │         │
│  │ else:                                 │         │
│  │    beat_cleaned[:,m] = beatM_full[:,m]│         │
│  └──────────────────────────────────────┘         │
└────────────────────────────────────────────────────┘

STEP 3: Performance Metrics
┌────────────────────────────────────────────────────┐
│  Removal statistics:                               │
│  ┌──────────────────────────────────────┐         │
│  │ n_removed: Number of nullified chirps │         │
│  │ η = n_removed / M (removal fraction)  │         │
│  │                                       │         │
│  │ Theoretical SNR gain:                 │         │
│  │ ΔSNR = -10·log₁₀(1 - η) [dB]         │         │
│  │                                       │         │
│  │ Example: η=30% → ΔSNR≈1.5 dB          │         │
│  └──────────────────────────────────────┘         │
│                                                    │
│  Evaluation metrics (optional, uses ground truth): │
│  ┌──────────────────────────────────────┐         │
│  │ Range error: |R_est - R_true| [m]     │         │
│  │ (for performance analysis only)       │         │
│  └──────────────────────────────────────┘         │
└────────────────────────────────────────────────────┘

OUTPUT:
┌────────────────────────────────────────────────────┐
│ beat_cleaned [Nsweep × M]                          │
│  • Nullified chirps set to zero                   │
│  • High-SNR chirps preserved                      │
│  • Reduced fluctuation impact                     │
│                                                    │
│ afs_metrics:                                       │
│  • n_removed, η (removal stats)                   │
│  • snr_gain_dB (theoretical)                      │
│  • range_error_m (eval only)                      │
│                                                    │
│ Ready for fusion with other radar band            │
└────────────────────────────────────────────────────┘
```

**AFS Rationale**:

In Swerling II targets, RCS nulls (deep fades) create chirps with SNR ≪ mean SNR. Including these in fusion **degrades** performance because:

1. **Noise dominance**: Low-RCS chirps are mostly noise
2. **Averaging dilution**: Mean of [good + noise] < good signal alone
3. **Phase corruption**: Random noise phases destroy coherent integration

AFS solution:
- **Detect** low-SNR chirps using CFAR
- **Remove** them from processing (set to zero)
- **Preserve** high-quality chirps for fusion
- **Result**: Effective SNR increase despite fewer samples

---

### Stage 9-10: Visualization (Beat Plots, RDM)

**Purpose**: Display processed beat signals and range-Doppler maps

```
Controlled by:
• PLOT_BEAT_WIDE_NAR
• PLOT_RANGE_DOPPLER

No data modification
```

---

### Stage 11: Save & Report

**Purpose**: Export processed data for downstream fusion

```
┌──────────────────────────────────────────────────────────────┐
│                  DATA EXPORT & REPORTING                     │
└──────────────────────────────────────────────────────────────┘

Saved Data (MAT file):
┌────────────────────────────────────────────────────┐
│ beat_5_8GHz.mat (R1) / beat_24GHz.mat (R2)         │
│                                                    │
│ Variables:                                         │
│  • beat_signal: x_wide [Nt × 1] complex           │
│  • fs_out: Sampling frequency (300 MHz)           │
│  • t_out: Time vector (matches beat_signal)       │
│  • swerling_data: Structure containing:           │
│    - enabled: true/false                          │
│    - mean_rcs_dBsm: Mean RCS value                │
│    - chirp_power_dB: Per-chirp power [M×1]        │
│    - chirp_power_dB_norm: Normalized power        │
│    - afs_data: AFS candidate information          │
└────────────────────────────────────────────────────┘

Console Report:
┌────────────────────────────────────────────────────┐
│ === FINAL ESTIMATE ===                             │
│ True R=37.00 m | fb_true=3.700 MHz                 │
│ Chosen -> fb=3.685 MHz | R=36.85 m                 │
│                                                    │
│ (if AFS enabled)                                   │
│ AFS Results:                                       │
│   - Removed chirps: 154 / 512 (η = 30.1%)          │
│   - Theoretical SNR gain: 1.56 dB                  │
│   - [EVAL] Range estimate error: 0.150 m           │
└────────────────────────────────────────────────────┘
```

---

## Data Flow Diagrams

### Complete End-to-End Signal Flow

```
CONFIG → PARAMS → WAVEFORM → TARGET → CHANNEL → NOISE → LNA
  ↓        ↓         ↓          ↓        ↓        ↓      ↓
 Set    Define   Generate   Swerling  Propagate  AWGN  Amplify
flags   specs     tx(t)      RCS        + FSPL   (pre)  (×1000)
                                                         
                                                         ↓
                                                      rx1d(t)
                                                         ↓
                  ┌──────────────────────────────────────┘
                  ↓
         RESHAPE → DECHIRP → SWERLING → UP-DETECT
             ↓         ↓          ↓          ↓
          [Ns×M]   beat=rx.*conj(tx)  ×α[m]  Find UP
                        ↓                     ↓
                   beatM_full            idx_up
                        ↓                     ↓
                        ├─────────────────────┤
                        ↓                     ↓
                  beatM_detect          beatM_plot
                   (all chirps)         (UP-only)
                        ↓                     ↓
                   SELECT SWEEP          RDM/VIZ
                   (N=1 or mid)         (plot-only)
                        ↓
                    TRIM EDGES
                        ↓
                  WIDE-IF FILTER
                        ↓
                  PHASE UNWRAP
                        ↓
                 PHASE-SLOPE FIT
                 (2-stage robust)
                        ↓
                   fb_est, R_est ←──────────┐
                        ↓                    │
                   (if AFS enabled)          │
                        ↓                    │
                     AFS CFAR                │
                  (range-cell detect)        │
                        ↓                    │
                   beat_cleaned              │
                        ↓                    │
                   SAVE TO FILE              │
                        ↓                    │
                   beat_5_8GHz.mat ──────────┘
                   (for fusion)
```

### Parallel Data Lanes (Detection vs Visualization)

```
                  beatM_full [Ns × M]
                         │
                         ├───────────────────────┐
                         ↓                       ↓
                  DETECTION LANE          VISUALIZATION LANE
                  (range estimate)        (plots, diagnostics)
                         │                       │
                    beatM_detect              beatM_plot
                    (all sweeps)              (UP-only)
                         │                       │
                  ┌──────┴────────┐              │
                  ↓               ↓              ↓
            N=1 mode         Multi-sweep    RDM/Spectro
            (single)         (mid+deDop)    (plot-only)
                  │               │              │
                  └───────┬───────┘              │
                          ↓                      │
                      x_raw_full                 │
                          │                      │
                     [PROCESSING]                │
                          │                      │
                          ↓                      │
                      R_est, fb_est              │
                          │                      │
                          └──────────────────────┘
                                     │
                                     ↓
                               SAVED OUTPUT
                               
CRITICAL: No feedback from plot lane to detection
          (prevents data leakage and overfitting)
```

---

## Mathematical Formulations

### Key Equations Summary

**1. FMCW Chirp Signal**:
$$s_{tx}(t) = \exp(j\pi\mu t^2), \quad \mu = \frac{B}{T_c}$$

**2. Beat Frequency**:
$$f_b = \mu \tau = \mu \cdot \frac{2R}{c} = \frac{2BR}{cT_c}$$

**3. Range from Beat Frequency**:
$$R = \frac{c f_b}{2\mu} = \frac{c f_b T_c}{2B}$$

**4. Doppler Frequency**:
$$f_d = \frac{2v}{\lambda} = \frac{2v f_c}{c}$$

**5. Radar Range Equation** (received power):
$$P_r = \frac{P_t G^2 \lambda^2 \sigma}{(4\pi)^3 R^4}$$

**6. Thermal Noise Power**:
$$N = kT_0 F B$$
where $k = 1.38 \times 10^{-23}$ J/K, $T_0 = 290$ K

**7. Swerling II Amplitude Distribution**:
$$|α|^2 \sim \text{Exponential}(\lambda = 1)$$
$$f(x) = e^{-x}, \quad x \geq 0$$

**8. Phase-Slope Estimator**:
$$\phi(t) = \beta_0 + \beta_1 t + \epsilon(t)$$
$$\beta = \arg\min_\beta \sum_i w_i (\phi_i - \beta_0 - \beta_1 t_i)^2$$

**9. AFS SNR Gain**:
$$\Delta\text{SNR}_\text{dB} = -10 \log_{10}(1 - \eta)$$
where $\eta$ = fraction of removed chirps

---

## Signal Characteristics at Each Stage

### Dimensional Evolution Table

| Stage | Signal Name | Dimensions | Type | Domain | Notes |
|-------|------------|------------|------|--------|-------|
| 2 | `tx` | [Ns_total × 1] | Complex | Baseband | Clean chirp train |
| 3 | `rx` | [Ns_total × 1] | Complex | Baseband | Echo (pre-noise, pre-LNA) |
| 3a | `rx1d` | [Ns_total × 1] | Complex | Baseband | Noisy, post-LNA |
| 6 | `txM` | [Nsweep × M] | Complex | Baseband | Tx matrix |
| 6 | `rxM` | [Nsweep × M] | Complex | Baseband | Rx matrix |
| 6 | `beatM_full` | [Nsweep × M] | Complex | Beat | Dechirped + Swerling |
| 8 | `x_raw_full` | [Nsweep × 1] | Complex | Beat | Selected sweep |
| 8 | `x0` | [Nt × 1] | Complex | Beat | Trimmed (80%) |
| 8 | `x_wide` | [Nt × 1] | Complex | Beat | IF-filtered |
| 8 | `phi` | [Nt × 1] | Real | Phase | Unwrapped phase (rad) |
| 8a | `beat_cleaned` | [Nsweep × M] | Complex | Beat | AFS-processed |
| 11 | `beat_signal` | [Nt × 1] | Complex | Beat | Saved for fusion |

where:
- `Ns_total` = M × Nsweep (total samples, all chirps)
- `M` = 512 (number of chirps)
- `Nsweep` ≈ 1000 (samples per chirp)
- `Nt` ≈ 800 (after trimming)

### Bandwidth Evolution

| Stage | Effective Bandwidth | Center Frequency | Reason |
|-------|-------------------|------------------|--------|
| Tx (Section 2) | 150 MHz | 0 Hz (baseband) | FMCW chirp |
| Rx (Section 3) | 150 MHz | 0 Hz (baseband) | Echo return |
| Beat (Section 6) | ~10 MHz | 3.7 MHz (fb) | Dechirp compression |
| Wide-IF (Section 8) | 50 MHz | 25 MHz (centered) | Hardware IF cap |
| Phase (Section 8) | ~10 kHz | DC (phase slope) | Unwrapped phase |

---

## Critical Design Decisions

### 1. Input-Referred Noise Model
**Decision**: Add AWGN **before** LNA gain (Section 3a)

**Rationale**:
- Models physical thermal noise at receiver input
- LNA amplifies signal+noise together (realistic RF chain)
- Prevents over-optimistic SNR simulation

**Alternative Rejected**: Post-LNA noise addition (disconnected from physics)

---

### 2. Swerling II Applied Post-Dechirp
**Decision**: Apply RCS fluctuation **after** dechirp (Section 6)

**Rationale**:
- MathWorks best practice for fluctuating targets
- Separates propagation physics (static RCS) from statistical model (amplitude scaling)
- Allows clean single-pass signal generation

**Alternative Rejected**: Multiple propagation passes (computationally expensive, equivalent result)

---

### 3. Single-Sweep Integrity Guard
**Decision**: N=1 mode uses exactly one sweep, with hash verification (Section 6)

**Rationale**:
- Prevents slow-time data leakage (Doppler/RDM never feeds detector)
- Ensures fair evaluation (no multi-sweep coherent gain)
- Hash check guarantees no accidental modification

**Implementation**:
```matlab
x_hash_before = sum(abs(x_raw_full).^2);
% [processing that must not modify x_raw_full]
x_hash_after = sum(abs(x_raw_full).^2);
assert(abs(x_hash_after - x_hash_before) < 1e-12);
```

---

### 4. Data Lane Separation
**Decision**: `beatM_detect` vs `beatM_plot` (Section 6)

**Rationale**:
- Detection path: pristine, no visualization artifacts
- Plot path: can use UP-only, RDM, etc. without contaminating detector
- Prevents overfitting to displayed data

**Key Principle**: Visualization never feeds back to estimation

---

### 5. Phase-Slope vs FFT
**Decision**: Use phase-slope as primary estimator (Section 8)

**Rationale**:
- Robust to noise (weighted least squares)
- Outlier rejection (IQR-based)
- Amplitude weighting (high-SNR samples prioritized)
- Hardware-consistent (IF bandpass applied)

**Jacobsen FFT Removed** (Feb 2026): Redundant cross-check eliminated

---

### 6. AFS Uses Estimated Range
**Decision**: AFS uses `R_est`, not `range_true` (Section 8a)

**Rationale**:
- **Honest mode**: Deployment-ready (no ground truth needed)
- Realistic evaluation (handles range estimation errors)
- Prevents cheating in performance metrics

**Evaluation Exception**: Ground truth can be passed for error analysis reporting, but NEVER used in detection logic

---

### 7. Wide-IF Constraint (50 MHz Cap)
**Decision**: Bandpass filter 0.1-50 MHz before estimation (Section 8)

**Rationale**:
- Models real hardware IF bandwidth limit
- Prevents unrealistic high-frequency processing
- Matches commercial FMCW radar front-ends

**Impact**: Slight range resolution degradation vs ideal infinite bandwidth

---

## Future Processing Stages (Pending Implementation)

### Phase 2: ICZT Transform Layer
```
[Current] FFT(beat) → range bins

[Planned] ICZT(beat, custom_grid) → arbitrary resolution
          Unified τₖ grid for both 5.8 GHz and 24 GHz
```

### Phase 3: Harmonic Shift Alignment
```
[Planned] 24 GHz echo × exp(j(ω₀,₂₄ - ω₀,₅.₈)τₖ)
          Aligns phases across 18.2 GHz spectral gap
```

### Phase 5: Fusion Matrix Construction
```
[Planned] Xₙc = [beat_5.8GHz_cleaned, beat_24GHz_cleaned]
          Joint processing for 168.2 MHz effective bandwidth
```

### Phase 6: Kalman Tracking
```
[Planned] SKF: State = [R, Ṙ]
          Sequential updates from both radar bands
          Smooth, continuous track despite RCS fluctuations
```

---

## Glossary of Key Variables

| Variable | Meaning | Units | Typical Value |
|----------|---------|-------|---------------|
| `fc` | Carrier frequency | Hz | 5.8 GHz / 24 GHz |
| `bw` | Sweep bandwidth | Hz | 150 MHz |
| `fs` | Sampling rate | Hz | 300 MHz |
| `sweep_time` | Chirp duration | s | 3.33 μs |
| `NumSweeps` | Number of chirps | - | 512 |
| `Nsweep` | Samples per chirp | - | ~1000 |
| `μ` (mu) | Chirp rate | Hz/s | 4.5e13 |
| `fb` | Beat frequency | Hz | 3.7 MHz (R=37m) |
| `fd` | Doppler frequency | Hz | ~967 Hz (v=50m/s) |
| `SNR_dB` | Input-referred SNR | dB | 0.11 / -4 |
| `LNA_gain` | Low-noise amp gain | linear | 1000 (60 dB) |
| `IF_FMAX` | IF bandwidth cap | Hz | 50 MHz |
| `R_est` | Estimated range | m | ~37 |
| `α` (alpha) | Swerling II amplitude | - | Complex, |α|²~Exp(1) |

---

**Document Version**: 1.0  
**Last Updated**: February 4, 2026  
**Maintained By**: Radar Fusion Research Team
