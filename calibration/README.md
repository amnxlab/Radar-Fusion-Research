# Calibration Module

This directory contains calibration procedures and reference measurements for establishing common reference planes between the two radar bands.

## Files

- `calibration_procedure.m` - Automated calibration routine using metallic plate targets
- `calibration_data.mat` - Stored calibration measurements and correction factors

## Purpose

Ensures both 5.8 GHz and 24 GHz radars perceive targets at identical physical distances by:
1. Measuring known reference targets (metallic plate at fixed distances)
2. Calculating time-delay offsets for each radar
3. Storing correction factors for runtime application

## Status

**Phase**: Implementation pending  
**Dependencies**: Phase 2 (ICZT and shared grid) must be completed first

## Calibration Targets

Recommended calibration points:
- 10m (near field)
- 50m (mid field)
- 90m (far field)

Target: Flat metallic plate (>0.5m² area) for strong, stable return
