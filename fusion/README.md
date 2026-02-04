# Fusion Module

This directory contains all multi-band radar fusion algorithms and processing functions.

## Files

- `grid_config.m` - Unified delay-axis grid configuration for both radar bands
- `fusion_alignment.m` - Harmonic shift for phase alignment across spectral gap
- `afs_noise_gate.m` - Adaptive Fusion Selection (AFS) with CFAR-based noise gating
- `fuzzy_weighting.m` - Fuzzy logic weighting functions for trust score calculation
- `construct_fusion_matrix.m` - Superposition matrix construction (X_nc)
- `fusion_estimator.m` - Phase-slope estimator applied to fused signal
- `kalman_tracker.m` - Sequential Kalman Filter (SKF) for track smoothing

## Status

**Phase**: Implementation pending  
**Dependencies**: Phase 1 (RCS modeling) and Phase 2 (ICZT) must be completed first

## Usage

Functions in this directory will be called by the main radar models (R1_model.m, R2_model.m) after individual signal processing is complete.
