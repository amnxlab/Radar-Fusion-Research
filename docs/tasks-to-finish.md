# Radar Fusion Research - Implementation Roadmap

**Project**: Multi-Band FMCW Radar Fusion for UAV Detection  
**Last Updated**: February 4, 2026  
**Status**: Phase 0 Complete (Foundation Corrections)

---

## Completed Tasks ✓

### Phase 0: Foundation Corrections (February 2026)
- [x] **Remove Jacobsen FFT Interpolation** - Eliminated redundant FFT-based estimator from both models
- [x] **Fix Input-Referred Noise Model** - Moved AWGN addition before LNA gain to properly model thermal noise amplification
- [x] **Documentation** - Created comprehensive technical documentation in `model_fixes_feb2026.md`

---

## Pending Implementation Tasks

### Phase 1: Target and Noise Modeling (Corrected Foundation)

**Goal**: Replace constant RCS with realistic fluctuation models and verify noise characteristics align with radar range equation.

#### Task 1.1: Implement Fluctuating RCS with Swerling II Model ✓
- [x] Replace constant RCS values in both models:
  - R1_model.m: `-17 dBsm` → Swerling II fluctuating (COMPLETED)
  - R2_model.m: `-9.5 dBsm` → Swerling II fluctuating (COMPLETED)
- [x] Simulate 20 dB amplitude fluctuations (UAV resonance region behavior)
- [x] Validate fluctuations across multiple chirp sweeps
- [x] Document physical justification for AFS stage

**Implementation Complete** (February 4, 2026):
- Created `targets/SwerlingIITarget.m` - Core Swerling II class with chi-squared(2) statistics
- Created `targets/DroneRCSModel.m` - UAV extension with multi-scatterer and elevation patterns
- Created `targets/run_swerling_demo.m` - Visualization and validation script
- Integrated into both R1 and R2 models
- Documentation in `docs/swerling_ii_model.md`

**Files Modified**: `R1_model.m`, `R2_model.m`  
**Files Created**: `targets/SwerlingIITarget.m`, `targets/DroneRCSModel.m`, `targets/run_swerling_demo.m`, `docs/swerling_ii_model.md`  
**Dependencies**: None  
**Physics Rationale**: UAVs in the resonance region exhibit significant RCS fluctuations; constant RCS models fail to capture this and don't justify the need for Adaptive Fusion Selection (AFS).

---

#### Task 1.2: Verify Input-Referred Noise Consistency
- [x] ~~Ensure noise added before LNA gain~~ (Completed Feb 4, 2026)
- [ ] Verify Monte Carlo noise realizations align with $N = kT_0FB$
- [ ] Cross-check SNR against monostatic radar range equation:
  $$P_r = \frac{P_t G^2 \lambda^2 \sigma}{(4\pi)^3 R^4}$$
- [ ] Adjust `SNR_dB` values if needed for realistic detection ranges
- [ ] Run sensitivity analysis: SNR vs detection probability

**Files to Verify**: `R1_model.m`, `R2_model.m` (Section 3)  
**Dependencies**: Task 1.1 (fluctuating RCS affects received power)  
**Estimated Effort**: 3-5 hours  
**Validation Metric**: Simulated SNR at 37m range matches theoretical prediction within ±2 dB

---

### Phase 2: The Transform Layer (Unified Mapping)

**Goal**: Replace standard FFT with ICZT to enable arbitrary resolution and create a unified delay-axis grid for both radar bands.

#### Task 2.1: Deploy Inverse Chirp-Z Transform (ICZT)
- [ ] Research MATLAB ICZT implementation (`czt` function or custom)
- [ ] Replace Range-FFT in both models with ICZT
- [ ] Define configurable resolution parameter (samples per meter)
- [ ] Validate ICZT output matches FFT for standard grid
- [ ] Benchmark computational cost vs FFT

**Files to Modify**: `R1_model.m`, `R2_model.m` (Section 8 - Range estimation)  
**Dependencies**: None  
**Estimated Effort**: 6-8 hours  
**Reference**: Rabiner, Schafer, Rader (1969) - "The Chirp z-Transform Algorithm"

---

#### Task 2.2: Define Shared Delay-Axis Grid
- [ ] Design unified grid $\tau_k$ spanning 0-100m range
- [ ] Apply ICZT with identical sampling to both 5.8 GHz and 24 GHz
- [ ] Eliminate dependency on matched radar parameters ($T_c$, $f_s$, $B$)
- [ ] Verify grid alignment: both radars map to same delay bins
- [ ] Document grid resolution vs computational cost trade-off

**Files to Modify**: Create `grid_config.m` or integrate into models  
**Dependencies**: Task 2.1  
**Estimated Effort**: 4-6 hours  
**Key Parameter**: Grid resolution (e.g., 1000 samples over 100m = 0.1m bins)

---

#### Task 2.3: Remove Jacobsen Interpolation Cross-Check
- [x] ~~Remove Jacobsen FFT code~~ (Completed Feb 4, 2026)
- [ ] Verify ICZT provides sub-bin resolution without interpolation
- [ ] Update documentation explaining why interpolation is obsolete
- [ ] Confirm range accuracy maintains or improves vs previous method

**Files to Verify**: `R1_model.m`, `R2_model.m`  
**Dependencies**: Task 2.1  
**Estimated Effort**: 1-2 hours  
**Rationale**: ICZT allows arbitrarily dense sampling; Jacobsen interpolation redundant

---

### Phase 3: Geometric Alignment (Gap Management)

**Goal**: Align 5.8 GHz and 24 GHz echoes across the 18.2 GHz spectral gap to create a virtual wideband signal.

#### Task 3.1: Apply Harmonic Shift for Phase Alignment
- [ ] Calculate carrier frequency offset: $\omega_{0,b} - \omega_{0,a}$ (24 GHz - 5.8 GHz)
- [ ] Multiply 24 GHz echo by $\exp(j(\omega_{0,b} - \omega_{0,a})\tau_k)$
- [ ] Verify phase continuity across spectral gap
- [ ] Validate alignment using synthetic targets at known ranges
- [ ] Document mathematical derivation

**Files to Create**: `fusion_alignment.m` (new function)  
**Dependencies**: Task 2.2 (shared grid required)  
**Estimated Effort**: 5-7 hours  
**Physics**: Aligns phases to simulate single 168.2 MHz effective bandwidth radar

---

#### Task 3.2: Establish Reference Planes via Calibration
- [ ] Design calibration measurement procedure (metallic plate)
- [ ] Collect reference data at known distance (e.g., 10m, 50m)
- [ ] Calculate per-radar time-delay offsets
- [ ] Apply calibration corrections to both models
- [ ] Validate: both radars report identical range to calibration target

**Files to Create**: `calibration_procedure.m`, `calibration_data.mat`  
**Dependencies**: Task 2.2  
**Estimated Effort**: 6-8 hours  
**Success Metric**: Range error < 0.1m for calibration target

---

### Phase 4: Robustness and Statistical Weighting

**Goal**: Implement adaptive logic to handle RCS fluctuations and assign trust scores to each radar stream.

#### Task 4.1: Implement AFS (Adaptive Fusion Selection) Logic ✓
**Status**: COMPLETED (February 4, 2026)

**Implementation**: AFS honest mode with CFAR-like sliding window detection
- [x] Design CFAR-based noise gate for range cells
- [x] Implement cell detection: compare detection cell vs reference cells
- [x] Nullify pulses where amplitude < threshold (RCS null condition)
- [x] Test with Swerling II fluctuations from Task 1.1
- [x] Tune CFAR parameters (guard cells, reference cells, threshold)
- [x] **HONEST MODE**: Uses estimated range (R_est) instead of ground truth
- [x] Validation mode for performance evaluation (range_true for metrics only)
- [x] Integration into R1_model.m and R2_model.m with toggle controls

**Files Created/Modified**: 
- `apply_AFS.m` (existing file updated to honest mode)
- `R1_model.m` (AFS integration with Section 8a)
- `R2_model.m` (AFS integration with Section 8a)
- `docs/afs_honest_plan.md` (design documentation)
- `docs/afs_code_changes_summary.md` (implementation guide)

**Key Feature**: AFS operates on **estimated** range from phase-slope method, making performance evaluation realistic and deployment-ready

**Usage**:
```matlab
% In R1_model.m or R2_model.m (Section 0: Toggles)
APPLY_AFS = true;              % Enable AFS processing
PLOT_AFS_PROCESSING = true;    % Show diagnostic plots
```

**Dependencies**: Task 1.1 ✓ (completed)  
**Actual Effort**: 3 hours (design + implementation + testing)  
**Reference**: Rohling (1983) - "Radar CFAR Thresholding in Clutter"

---

#### Task 4.2: Apply Fuzzy Weighting Functions
- [ ] Replace fixed 0.35/0.65 weights with dynamic fuzzy logic
- [ ] Design membership functions based on local clutter/SNR
- [ ] Calculate per-sweep "trust" scores for each radar
- [ ] Implement weighting: $w_k = f(\text{SNR}_k, \text{clutter}_k)$
- [ ] Validate: high-SNR sweeps receive higher weights

**Files to Create**: `fuzzy_weighting.m`  
**Dependencies**: Task 4.1  
**Estimated Effort**: 6-8 hours  
**Reference**: Zadeh (1965) - "Fuzzy Sets"; Dong et al. (2016) - Fuzzy radar fusion

---

### Phase 5: Fusion and Final Estimation

**Goal**: Combine weighted, aligned data streams into unified high-resolution signal and extract range estimate.

#### Task 5.1: Construct Superposition Matrix $X_{nc}$
- [ ] Stack cleaned and weighted data from both radars
- [ ] Create unified matrix: rows = time samples, columns = radar bands
- [ ] Apply AFS nullification masks from Task 4.1
- [ ] Apply fuzzy weights from Task 4.2
- [ ] Verify matrix dimensions and alignment

**Files to Create**: `construct_fusion_matrix.m`  
**Dependencies**: Tasks 3.1, 3.2, 4.1, 4.2  
**Estimated Effort**: 4-6 hours  
**Output**: Combined bandwidth signal without phase synchronization hardware

---

#### Task 5.2: Re-apply Phase-Slope Estimator to Fused Signal
- [ ] Extract fused beat signal from $X_{nc}$
- [ ] Apply existing robust two-stage phase-slope fit
- [ ] Compare range accuracy: fused vs single-band
- [ ] Measure resolution improvement from combined 168.2 MHz bandwidth
- [ ] Document performance gain

**Files to Modify**: Integrate into models or create `fusion_estimator.m`  
**Dependencies**: Task 5.1  
**Estimated Effort**: 3-5 hours  
**Expected**: ~0.89m range resolution (vs ~1m single-band)

---

### Phase 6: Tracking and Evaluation (Success Metrics)

**Goal**: Stabilize estimates with Kalman filtering and evaluate system performance using research-grade metrics.

#### Task 6.1: Implement Sequential Kalman Filter (SKF)
- [ ] Design SKF state vector: $[R, \dot{R}]$ (range, range rate)
- [ ] Define process model (constant velocity or acceleration)
- [ ] Define measurement model (noisy range observations)
- [ ] Tune process noise $Q$ and measurement noise $R$ covariances
- [ ] Integrate both radar bands as sequential sensor updates
- [ ] Validate: smooth, continuous track output

**Files to Create**: `kalman_tracker.m`  
**Dependencies**: Task 5.2  
**Estimated Effort**: 8-10 hours  
**Reference**: Kalman (1960); Bar-Shalom et al. (2001) - Tracking textbook

---

#### Task 6.2: Calculate Continuous Performance Metrics
- [ ] Implement Consecutive Block Detection (CBD) metric
- [ ] Implement Intersection-over-Union (IoU) metric
- [ ] Penalize track interruptions from RCS nulls
- [ ] Penalize misalignments from spectral gap
- [ ] Generate performance report: CBD%, IoU%, track continuity
- [ ] Compare: fused system vs single-band baseline

**Files to Create**: `evaluate_performance.m`  
**Dependencies**: Task 6.1  
**Estimated Effort**: 6-8 hours  
**Success Criteria**: 
- CBD > 95% (track maintained across fluctuations)
- IoU > 0.90 (aligned with ground truth)
- Track interruptions < 5% of total time

---

## Project Structure

```
Radar-Fusion-Research/
├── R1_model.m              # 5.8 GHz FMCW model (foundation)
├── R2_model.m              # 24 GHz FMCW model (foundation)
├── docs/
│   ├── model_fixes_feb2026.md    # Completed corrections
│   └── tasks-to-finish.md        # This file
├── fusion/                  # (To be created)
│   ├── grid_config.m
│   ├── fusion_alignment.m
│   ├── afs_noise_gate.m
│   ├── fuzzy_weighting.m
│   ├── construct_fusion_matrix.m
│   ├── fusion_estimator.m
│   └── kalman_tracker.m
├── calibration/             # (To be created)
│   ├── calibration_procedure.m
│   └── calibration_data.mat
└── evaluation/              # (To be created)
    └── evaluate_performance.m
```

---

## Implementation Timeline (Estimated)

| Phase | Tasks | Estimated Hours | Priority |
|-------|-------|-----------------|----------|
| **Phase 1** | Target/Noise Modeling | 5-9 hrs | **HIGH** |
| **Phase 2** | Transform Layer (ICZT) | 11-16 hrs | **HIGH** |
| **Phase 3** | Geometric Alignment | 11-15 hrs | **MEDIUM** |
| **Phase 4** | AFS & Fuzzy Weighting | 14-18 hrs | **MEDIUM** |
| **Phase 5** | Fusion & Estimation | 7-11 hrs | **HIGH** |
| **Phase 6** | Tracking & Metrics | 14-18 hrs | **HIGH** |
| **Total** | | **62-87 hrs** | |

**Recommended Order**: Phase 1 → Phase 2 → Phase 5 (basic fusion) → Phase 3 (alignment) → Phase 4 (robustness) → Phase 6 (evaluation)

---

## Key References

1. **Chirp-Z Transform**: Rabiner, Schafer, Rader (1969) - IEEE Trans ASSP
2. **Swerling Models**: Swerling (1960) - "Probability of Detection for Fluctuating Targets"
3. **CFAR Detection**: Rohling (1983) - "Radar CFAR Thresholding in Clutter"
4. **Fuzzy Logic**: Zadeh (1965); Dong et al. (2016) - Fuzzy sensor fusion
5. **Kalman Filtering**: Kalman (1960); Bar-Shalom (2001) - "Estimation with Applications to Tracking"
6. **Radar Range Equation**: Skolnik (2008) - "Radar Handbook"

---

## Notes

- **Critical Path**: Phases 1-2 must be completed before any fusion work
- **Testing Strategy**: Validate each phase with synthetic targets before moving to next
- **Hardware Validation**: Once simulation complete, compare with real UAV measurements
- **Publication Target**: IEEE Transactions on Aerospace and Electronic Systems

---

**Next Immediate Action**: Begin Task 1.1 - Implement Swerling II RCS model
