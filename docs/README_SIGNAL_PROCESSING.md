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
sequenceDiagram
    participant TX as Transmitter
    participant CH as Channel
    participant TGT as Target
    participant RX as Receiver

    Note over TX: Generate FMCW Chirp
    Note over TX: s_tx(t) = exp(j*pi*mu*t^2)
    TX->>CH: Transmit Signal
    
    Note over CH: Free Space Path Loss (1/R^4)
    CH->>TGT: Incident Wave
    
    Note over TGT: Reflection
    Note over TGT: RCS (Mean)
    TGT->>CH: Reflected Wave
    
    Note over CH: Return Path Loss
    CH->>RX: Received Signal (Clean)
    Note over RX: s_rx = A * s_tx(t-tau)
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

This stage converts the time-domain signal into the beat-frequency domain and applies statistical RCS fluctuations.

```mermaid
graph TB
    %% Inputs
    RX1D[rx1d Input Vector]:::signal
    TX1D[tx1d Input Vector]:::signal
    
    %% reshaping
    Reshape[Reshape to Matrix<br>Nsweep x M]:::process
    
    %% Dechirp
    DechirpNode[Dechirp Operation<br>beat = rx * conj_tx]:::process
    
    %% Swerling Branch
    SwerlingGen[Swerling II Generator]:::physics
    Alpha[Generate Alpha Gain<br>|alpha|^2 ~ Exp_1]:::physics
    
    %% Mixing
    ApplyFluct[Apply Fluctuation<br>beatM * alpha]:::process
    
    %% Output
    BeatFull[beatM_full Matrix]:::signal

    %% Connections
    RX1D --> Reshape
    TX1D --> Reshape
    Reshape --> DechirpNode
    DechirpNode --> ApplyFluct
    SwerlingGen --> Alpha --> ApplyFluct
    ApplyFluct --> BeatFull
    
    %% Styles - High Contrast
    classDef signal fill:#BBDEFB,stroke:#333,stroke-width:1px,color:black;
    classDef process fill:#E1BEE7,stroke:#333,stroke-width:1px,color:black;
    classDef physics fill:#FFF9C4,stroke:#333,stroke-width:1px,color:black;
```

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
