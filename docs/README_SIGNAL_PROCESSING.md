# FMCW Radar Signal Processing Chain - Visual Documentation

**Project**: Multi-Band Radar Fusion Research  
**Date**: February 4, 2026  
**Source**: `docs/signal_processing_chain.md` & `docs/model_fixes_feb2026.md`

This document provides a detailed visual breakdown of the signal processing interactions for the 5.8 GHz and 24 GHz FMCW radar models.

---

## 1. High-Level System Overview

The following diagram illustrates the complete end-to-end data flow from configuration to final range estimation.

```mermaid
graph TD
    %% Define Styles - High Contrast
    classDef config fill:#FFCCBC,stroke:#333,stroke-width:2px,color:black;
    classDef signal fill:#BBDEFB,stroke:#333,stroke-width:2px,color:black;
    classDef process fill:#E1BEE7,stroke:#333,stroke-width:2px,color:black;
    classDef physics fill:#FFF9C4,stroke:#333,stroke-width:2px,color:black;
    classDef output fill:#C8E6C9,stroke:#333,stroke-width:2px,color:black;

    subgraph "Initialization & Physics"
        Config[Configuration & Toggles]:::config --> Params[System Constants]:::config
        Params --> Waveform[Waveform Generation]:::process
        Waveform -->|tx| Channel[Channel Propagation]:::physics
        Channel -->|rx clean| FrontEnd[RF Front-End]:::physics
    end

    subgraph "Receiver Processing"
        FrontEnd -->|rx1d| Dechirp[Dechirp & Reshape]:::process
        Dechirp -->|beatM| Swerling[Swerling II Fluctuation]:::physics
        Swerling -->|beatM_full| DetectSplit{Lane Split}:::process
    end

    subgraph "Estimation Path"
        DetectSplit -->|Detection| SweepSel[Sweep Selection]:::process
        SweepSel --> WideIF[Wide-IF Filter]:::process
        WideIF --> PhaseFit[Phase-Slope Estimator]:::process
        PhaseFit -->|R_est| AFS[AFS Processing]:::process
    end

    subgraph "Visualization Path"
        DetectSplit -->|Plotting| UPDetect[UP-Sweep Detection]:::process
        UPDetect --> Plots[RDM & Spectrograms]:::output
    end
    
    AFS -->|beat_cleaned| Save[Save & Export]:::output

    linkStyle default stroke-width:2px;
```

---

## 2. Detailed Processing Stages

### Stage A: Waveform Generation & Propagation

The signal originates as a perfect baseband chirp and propagates through the environment, picking up delay and attenuation.

```mermaid
graph LR
    %% Styles - High Contrast
    classDef signal fill:#BBDEFB,stroke:#333,stroke-width:1px,color:black;
    classDef process fill:#E1BEE7,stroke:#333,stroke-width:1px,color:black;
    classDef physics fill:#FFF9C4,stroke:#333,stroke-width:1px,color:black;
    
    subgraph "1. Waveform Source"
        Gen[FMCW Generator<br>B=150MHz, T=3.33us]:::process
        Eq[Signal Model<br>exp j_pi_mu_t^2]:::process
    end

    subgraph "2. Transmission"
        TX_Ant[TX Antenna<br>Gain = +30dB]:::physics
        Prop1[Forward Path<br>Loss = 1/R^4]:::physics
    end

    subgraph "3. Target Interaction"
        Target[UAV Target<br>RCS Re-radiation]:::physics
    end

    subgraph "4. Reception"
        Prop2[Return Path<br>Delay = 2R/c]:::physics
        RX_Ant[RX Antenna<br>Capture Echo]:::physics
        Signal[RX Signal<br>s_rx t]:::signal
    end

    %% Flow
    Gen --> Eq --> TX_Ant
    TX_Ant -->|Radiated Wave| Prop1
    Prop1 -->|Incident Wave| Target
    Target -->|Reflected Wave| Prop2
    Prop2 -->|Echo| RX_Ant
    RX_Ant --> Signal
```

### Stage B: RF Front-End (Input-Referred Noise)

**CRITICAL**: Noise is added **before** amplification to correctly model physical thermal noise.

```mermaid
graph LR
    %% Nodes
    RX_Clean(RX Clean Signal):::signal
    Noise(Thermal Noise Gen):::physics
    K_CONST(k*T0*B):::config
    Adder((+)):::process
    LNA[LNA Amplifier 60dB]:::process
    RX_Out(RX Output):::signal
    
    %% Styles - High Contrast
    classDef signal fill:#BBDEFB,stroke:#333,stroke-width:1px,color:black;
    classDef process fill:#E1BEE7,stroke:#333,stroke-width:1px,color:black;
    classDef physics fill:#FFF9C4,stroke:#333,stroke-width:1px,color:black;
    classDef config fill:#FFCCBC,stroke:#333,stroke-dasharray: 5 5,color:black;

    %% Flows
    RX_Clean --> Adder
    K_CONST --> Noise
    Noise -->|AWGN| Adder
    Adder -->|Signal + Noise| LNA
    LNA -->|Amplified Sig + Amp Noise| RX_Out

    %% Descriptions
    subgraph "Physics-Correct Model"
    direction LR
    Adder
    LNA
    end
```

### Stage C: Dechirp & Swerling II Model

This stage performs three critical functions: 
1.  **Dechirping**: Compresses the wideband chirp into a narrowband beat frequency.
2.  **Swerling II Physics**: Applies frame-to-frame RCS fluctuations (decorrelated per chirp).
3.  **Lane Splitting**: Separates data for processing (all chirps) vs visualization (UP-chirps only).

```mermaid
graph TD
    %% Define inputs
    RX[RX Signal Vector]:::signal
    TX[TX Signal Vector]:::signal
    
    subgraph "1. Matrix Reshaping"
        Reshape[Reshape 1D to Matrix<br>Fast-Time x Slow-Time]:::process
    end
    
    subgraph "2. Dechirp (Pulse Compression)"
        Conj{Conjugate TX}:::math
        Mix((X)):::math
        BeatRaw[Raw Beat Matrix]:::signal
    end
    
    subgraph "3. Swerling II Physics (RCS)"
        GenGauss[Generate Complex Gaussian<br>Re, Im ~ N 0,1]:::physics
        Power[Calc Power & Normalize<br>abs_alpha^2 ~ Exp Dist]:::physics
        ApplyAlpha[Apply Phasor<br>beat * alpha_m]:::process
    end
    
    subgraph "4. Data Routing (Lane Split)"
        BeatFull[beatM_full]:::signal
        Split{Split}:::process
        DetectLane[Detection Lane<br>All Chirps]:::output
        PlotLane[Plotting Lane<br>UP-Sweeps Only]:::output
    end

    %% Flow
    RX --> Reshape
    TX --> Reshape
    Reshape -->|rxM / txM| Mix
    Reshape -->|txM| Conj
    Conj -->|tx*| Mix
    Mix -->|rx * tx*| BeatRaw
    
    BeatRaw --> ApplyAlpha
    GenGauss --> Power -->|alpha vectors| ApplyAlpha
    
    ApplyAlpha --> BeatFull
    BeatFull --> Split
    Split -->|Full Data| DetectLane
    Split -->|Subset| PlotLane

    %% Specific Styles for this graph
    classDef math fill:#FFECB3,stroke:#FF6F00,stroke-width:2px;
    classDef output fill:#C8E6C9,stroke:#333,stroke-width:2px;
```

**Physics Note**: The "Dechirp" operation ($RX \cdot TX^*$) mathematically extracts the range delay as a frequency shift ($f_b$). The Swerling model then modulates this clean signal with a stochastic complex gain ($\alpha$) to simulate a fluctuating target cross-section.

### Stage D: Range Estimation (Phase-Slope Method)

The primary estimator uses the phase slope of the beat signal, which is more robust than FFT peak picking for short-range FMCW.

```mermaid
graph TD
    %% Input
    Input[beatM_detect]:::signal
    
    %% Steps
    subgraph "Sweep Processing"
        Sel{Single or Multi?}:::process
        Single[Select Fixed Sweep]:::process
        Multi[Select Middle UP-Sweep]:::process
        Trim[Edge Trimming<br>Keep 10-90%]:::process
    end
    
    subgraph "Filtering"
        BPF[Wide-IF Bandpass Filter<br>0.1 - 50 MHz]:::process
    end
    
    subgraph "Estimation Logic"
        Unwrap[Phase Unwrap]:::process
        RobustFit[Robust Linear Fit<br>Weighted Least Squares]:::process
        Calc[Calc Range from Slope<br>R = c*fb / 2*mu]:::process
    end
    
    %% Output
    Result(R_est, fb_est):::output
    
    %% Flows
    Input --> Sel
    Sel -->|Single| Single
    Sel -->|Multi| Multi
    Single --> Trim
    Multi --> Trim
    Trim --> BPF
    BPF --> Unwrap
    Unwrap --> RobustFit
    RobustFit --> Calc
    Calc --> Result

    %% Styles - High Contrast
    classDef signal fill:#BBDEFB,stroke:#333,stroke-width:1px,color:black;
    classDef process fill:#E1BEE7,stroke:#333,stroke-width:1px,color:black;
    classDef output fill:#C8E6C9,stroke:#333,stroke-width:1px,color:black;
```

### Stage E: Adaptive Fusion Selection (AFS)

AFS filters out chirps that have low SNR due to RCS nulls (deep fades), preparing the data for clean fusion.

```mermaid
graph LR
    %% Inputs
    InBeat[beatM_full]:::signal
    RangeEst[R_est]:::signal
    
    %% Logic
    Goal(For Each Chirp...):::process
    
    CFAR[CFAR Detection<br>Calc SNR at R_est]:::process
    Threshold{SNR < SNR_mean - 6dB?}:::process
    
    Null[Nullify Chirp<br>Set to 0]:::process
    Keep[Keep Chirp]:::process
    
    OutBeat[beat_cleaned]:::output
    
    %% Flow
    InBeat --> Goal
    RangeEst --> Goal
    Goal --> CFAR
    CFAR --> Threshold
    Threshold -- Yes --> Null
    Threshold -- No --> Keep
    Null --> OutBeat
    Keep --> OutBeat
    
    %% Styles - High Contrast
    classDef signal fill:#BBDEFB,stroke:#333,stroke-width:1px,color:black;
    classDef process fill:#E1BEE7,stroke:#333,stroke-width:1px,color:black;
    classDef output fill:#C8E6C9,stroke:#333,stroke-width:1px,color:black;
```

---

## 3. Data Flow & Dimensions

The following diagram tracks the shape and type of the signal as it moves through the pipeline.

```mermaid
classDiagram
    class TimeDomain {
        tx : [Ns_total x 1] Complex
        rx : [Ns_total x 1] Complex
        rx1d : [Ns_total x 1] Complex
    }
    
    class MatrixDomain {
        txM : [Nsweep x M] Complex
        rxM : [Nsweep x M] Complex
        beatM_full : [Nsweep x M] Complex
    }
    
    class EstDomain {
        x_raw : [Nsweep x 1] Complex
        x_wide : [Nt x 1] Complex
        phi : [Nt x 1] Real
    }
    
    class Output {
        beat_signal : [Nt x 1] Complex
        R_est : Scatter
    }
    
    TimeDomain --> MatrixDomain : Reshape
    MatrixDomain --> EstDomain : Slice & Filter
    EstDomain --> Output : Save
```

***

**End of Visual Documentation**
