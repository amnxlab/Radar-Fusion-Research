# Swerling II Target Model Documentation

**Project**: Multi-Band FMCW Radar Fusion for UAV Detection  
**Created**: February 4, 2026  
**Status**: Implementation Complete

---

## Overview

This document describes the implementation of the Swerling II fluctuating RCS model for the dual-band FMCW radar simulation. The model replaces static RCS values with realistic pulse-to-pulse fluctuations essential for simulating UAV detection in the resonance region.

## Theory

### Swerling Model Classification

| Type | Statistical Model | Fluctuation Rate | Use Case |
|------|-------------------|------------------|----------|
| **Swerling I** | Chi-squared (2 DOF) | Scan-to-scan | Slow targets |
| **Swerling II** | Chi-squared (2 DOF) | **Pulse-to-pulse** | **UAVs, fast targets** |
| Swerling III | Chi-squared (4 DOF) | Scan-to-scan | Dominant scatterer |
| Swerling IV | Chi-squared (4 DOF) | Pulse-to-pulse | Dominant scatterer |

### Chi-Squared (2 DOF) Distribution

The RCS probability density function is:

$$p(\sigma) = \frac{1}{\bar{\sigma}} \exp\left(-\frac{\sigma}{\bar{\sigma}}\right), \quad \sigma \geq 0$$

This is equivalent to an **exponential distribution** with mean = $\bar{\sigma}$.

Key properties:
- **Mean**: $E[\sigma] = \bar{\sigma}$
- **Standard deviation**: $\sigma_{\sigma} = \bar{\sigma}$ (equal to mean)
- **Median**: $\sigma_{50\%} = \bar{\sigma} \ln(2) \approx 0.693 \bar{\sigma}$
- **Fluctuation range**: Typically 15-25 dB (99th percentile vs 1st percentile)

### Complex Amplitude Model

For each chirp $m$, the complex scattering amplitude is generated as:

$$\alpha_m = X_m + jY_m$$

where $X_m, Y_m \sim \mathcal{N}(0, \bar{\sigma}/2)$ are independent Gaussian random variables.

This ensures:
- $|\alpha_m|^2 \sim \text{Exponential}(\bar{\sigma})$
- Phase is uniformly distributed: $\theta_m = \arg(\alpha_m) \sim \mathcal{U}(0, 2\pi)$
- Independence between consecutive pulses (IID)

---

## Implementation

### File Structure

```
Radar-Fusion-Research/
├── targets/
│   ├── SwerlingIITarget.m      # Core Swerling II class
│   ├── DroneRCSModel.m         # UAV-specific extension
│   └── run_swerling_demo.m     # Visualization & validation
├── R1_model.m                  # 5.8 GHz (modified)
├── R2_model.m                  # 24 GHz (modified)
└── docs/
    └── swerling_ii_model.md    # This file
```

### SwerlingIITarget Class

Core class implementing Swerling II fluctuating RCS:

```matlab
% Create target with mean RCS = -17 dBsm, 512 chirps, 5.8 GHz
target = SwerlingIITarget(-17, 512, 5.8e9, 'shuffle');

% Generate fluctuating RCS
[alpha, rcs_m2] = target.generateFluctuatingRCS();

% Get AFS data for downstream processing
afs_data = target.getAFSData(baseline_snr_dB);

% Visualize
target.plotRCSPattern();
target.plotHistogram();
target.validateDistribution();
```

**Key Methods**:
- `generateFluctuatingRCS()` - Generate M×1 complex amplitudes and RCS values
- `getAFSData(snr)` - Get structure for Amplitude Fluctuation Suppression
- `plotRCSPattern()` - Visualize pulse-to-pulse RCS fluctuations
- `plotHistogram()` - Compare simulated vs theoretical chi-squared
- `validateDistribution()` - Kolmogorov-Smirnov test for statistical validity

### DroneRCSModel Class

Extended class for UAV-specific modeling:

```matlab
% Create quadrotor drone with 5 scattering centers
drone = DroneRCSModel(-17, 512, 5.8e9, 'shuffle', ...
    'DroneType', 'quadrotor', 'NumScatterers', 5);

% Generate RCS (coherent sum of scatterers)
[alpha, rcs] = drone.generateFluctuatingRCS();

% Additional visualizations
drone.plotExtendedScattererPattern();
drone.plotRCSvsElevation();
```

**Features**:
- Multi-scatterer spatial distribution
- Support for 'generic', 'quadrotor', 'hexarotor' drone types
- RCS vs elevation angle modeling
- Coherent scatterer summation

---

## Dual-Band Configuration

### RCS Parameters

| Parameter | 5.8 GHz (R1) | 24 GHz (R2) | Notes |
|-----------|--------------|-------------|-------|
| Mean RCS | -17 dBsm | -9.5 dBsm | 7.5 dB difference |
| Mean RCS | 0.020 m² | 0.112 m² | Linear scale |
| Wavelength | 51.7 mm | 12.5 mm | λ scaling |

### SNR Offset Calculation

The net SNR offset between bands accounts for both RCS and wavelength:

```
RCS offset:        +7.5 dB  (24 GHz has higher RCS)
λ² offset:        -12.3 dB  (24 GHz has shorter wavelength)
────────────────────────────
Net SNR offset:   -4.85 dB  (5.8 GHz has higher net SNR)
```

This is preserved in the simulation by using the specified mean RCS values.

---

## AFS (Amplitude Fluctuation Suppression) Interface

The Swerling II implementation provides data for AFS processing:

```matlab
afs_data = target.getAFSData(baseline_snr_dB);
```

**Output structure**:
| Field | Description |
|-------|-------------|
| `rcs_m2` | [M×1] Instantaneous RCS in m² |
| `rcs_dB` | [M×1] RCS deviation from mean (dB) |
| `complex_alpha` | [M×1] Complex scattering amplitudes |
| `local_snr_dB` | [M×1] Local SNR = baseline + RCS deviation |
| `low_snr_mask` | [M×1] Logical mask for nullification |
| `threshold_dB` | Threshold used (baseline - 6 dB) |
| `pct_low_snr` | Percentage of pulses below threshold |

**Usage in AFS**:
```matlab
% Nullify low-SNR pulses before fusion
beat_signal_afs = beat_signal;
beat_signal_afs(afs_data.low_snr_mask) = 0;  % Or set to NaN
```

---

## Integration with Radar Models

### R1_model.m (5.8 GHz)

Modified Section 3 to include Swerling II:

```matlab
% Create Swerling II target model
swerling_target = SwerlingIITarget(-17, NumSweeps, fc, RNG_MODE);
[alpha_fluctuating, rcs_fluctuating] = swerling_target.generateFluctuatingRCS();

% Use phased.RadarTarget with Swerling II model enabled
target = phased.RadarTarget('MeanRCS', swerling_target.MeanRCS_m2, ...
    'OperatingFrequency', fc, 'Model', 'Swerling2');
```

### R2_model.m (24 GHz)

Same pattern with -9.5 dBsm mean RCS:

```matlab
swerling_target = SwerlingIITarget(-9.5, NumSweeps, fc, RNG_MODE);
```

---

## Validation

### Statistical Tests

1. **Kolmogorov-Smirnov Test**:
   ```matlab
   [h, p] = target.validateDistribution();
   % h=0, p>0.05 indicates good fit to chi-squared(2)
   ```

2. **Mean RCS Convergence**:
   ```matlab
   stats = target.getStatistics();
   assert(stats.mean_error_pct < 5, 'Mean RCS drift exceeded');
   ```

3. **Independence Check** (autocorrelation near zero):
   ```matlab
   acf = autocorr(rcs_m2, 'NumLags', 10);
   assert(all(abs(acf(2:end)) < 0.1), 'Not IID');
   ```

### Visual Verification

Run the demonstration script:
```matlab
cd targets
run_swerling_demo
```

This generates 10 plots covering all aspects of the Swerling II implementation.

---

## Noise Model Verification

The existing noise model correctly implements input-referred AWGN:

```matlab
% Noise added BEFORE LNA (correct)
rx1d = awgn(rx, SNR_dB, 'measured');

% LNA amplifies signal + noise together
rx1d = rx1d * LNA_gain;
```

With Swerling II:
- **Signal power fluctuates** with RCS (varies pulse-to-pulse)
- **Noise power remains stable** (constant kT₀FB)
- **Local SNR varies** = baseline SNR + RCS deviation

This is essential for realistic simulation where target signals can drop below the noise floor during RCS nulls.

---

## Physics Rationale

### Why Swerling II for UAVs?

1. **Resonance Region**: Small UAVs (10-50 cm) at 5.8-24 GHz operate in the resonance region where λ ≈ target dimensions

2. **Complex Scattering**: Multiple scattering centers (body, motors, arms, propellers) with comparable RCS contributions

3. **Fast Fluctuation**: UAV motion, propeller rotation, and slight attitude changes cause pulse-to-pulse RCS variation

4. **Detection Challenges**: Without fluctuation modeling, simulations overestimate detection probability and don't capture RCS null events

### Justification for AFS

Swerling II fluctuations cause:
- ~5-10% of pulses to be 10+ dB below mean
- ~1-2% of pulses to be 20+ dB below mean (near noise floor)

AFS identifies and nullifies these low-SNR pulses before fusion to prevent:
- Noise injection into fused signal
- False target splitting
- Range estimation bias

---

## References

1. Swerling, P. (1960). "Probability of Detection for Fluctuating Targets", IRE Transactions on Information Theory

2. MathWorks: [Swerling Target Models](https://www.mathworks.com/help/phased/ug/swerling-2-target-models.html)

3. Skolnik, M.I. (2008). "Radar Handbook", 3rd Edition, Chapter 7

4. Richards, M.A. (2014). "Fundamentals of Radar Signal Processing", 2nd Edition

---

## Changelog

| Date | Change |
|------|--------|
| Feb 4, 2026 | Initial implementation of SwerlingIITarget class |
| Feb 4, 2026 | Added DroneRCSModel extension |
| Feb 4, 2026 | Integrated into R1_model.m and R2_model.m |
| Feb 4, 2026 | Created run_swerling_demo.m visualization script |
