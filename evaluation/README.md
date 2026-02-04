# Evaluation Module

This directory contains performance evaluation metrics and analysis tools for the multi-band fusion system.

## Files

- `evaluate_performance.m` - Comprehensive performance analysis suite
  - Consecutive Block Detection (CBD) metric
  - Intersection-over-Union (IoU) metric
  - Track continuity analysis
  - RCS fluctuation resilience testing

## Metrics

### Consecutive Block Detection (CBD)
Measures percentage of time the tracking system maintains lock on the target despite RCS fluctuations.
- **Target**: >95%

### Intersection-over-Union (IoU)
Quantifies alignment accuracy between estimated track and ground truth.
- **Target**: >0.90

### Track Interruption Rate
Percentage of time where track is lost due to deep RCS nulls.
- **Target**: <5%

## Status

**Phase**: Implementation pending  
**Dependencies**: Phase 6 (Kalman tracking) must be completed first

## Usage

Run after fusion system is complete to generate performance reports comparing:
- Fused system vs single-band (5.8 GHz only)
- Fused system vs single-band (24 GHz only)
- Impact of AFS and fuzzy weighting
