# System Architecture

This document describes the end-to-end architecture of the EMG-controlled assistive hand exoskeleton, covering signal flow, hardware partitioning, firmware structure, and safety considerations.

---

## 1. High-Level Architecture

The system is organized into four primary layers:

1. **Human Interface Layer** – EMG signal acquisition from the user
2. **Signal Conditioning Layer** – Analog amplification, rectification, and filtering
3. **Embedded Control Layer** – Real-time processing and control logic
4. **Mechanical Actuation Layer** – Servo-driven four-bar linkage for finger motion

```
User Muscle Activation
         ↓
Surface EMG Electrodes
         ↓
Analog EMG Conditioning
         ↓
Arduino Nano (ADC + Control)
         ↓
PWM Servo Command
         ↓
Four-Bar Linkage
         ↓
Assisted Finger Flexion
```

---

## 2. Human Interface Layer (EMG Input)

### 2.1 Signal Source
- Surface electromyography (sEMG) from forearm flexor muscles
- Electrodes placed over flexor digitorum region
- Single-channel acquisition used for robustness and simplicity

### 2.2 Design Rationale
- Surface EMG provides early intent detection (pre-motion)
- Single-channel approach reduces:
  - Calibration complexity
  - Sensor placement sensitivity
  - Processing overhead

---

## 3. Signal Conditioning Layer (Analog Front-End)

The EMG signal conditioning is performed externally using an analog module before digitization.

### 3.1 Signal Characteristics
- Raw EMG amplitude: ~0.2–1.2 mV
- Noise sources:
  - Power-line interference
  - Motion artifacts
  - Skin impedance variation

### 3.2 Conditioning Stages

1. **Differential Instrumentation Amplifier**
   - High CMRR
   - Suppresses common-mode noise
   - Converts microvolt-level signals to usable range

2. **Precision Full-Wave Rectification**
   - Converts bipolar EMG waveform to unipolar signal
   - Preserves signal magnitude independent of polarity

3. **Low-Pass Filtering / Envelope Detection**
   - Produces smooth EMG envelope
   - Typical cutoff ≈ 5 Hz
   - Suitable for proportional control

4. **Level Shifting**
   - Output constrained to 0–5 V
   - Direct compatibility with Arduino ADC

---

## 4. Embedded Control Layer (Arduino Nano)

### 4.1 Microcontroller Role
The Arduino Nano acts as the central controller responsible for:
- Sampling conditioned EMG signals
- Digital filtering and normalization
- Calibration management
- Servo command generation
- Fault detection and safety handling

---

### 4.2 Timing Architecture

The firmware uses decoupled timing loops:

| Task              | Rate        |
|-------------------|------------|
| EMG Sampling      | ~1 kHz     |
| Servo Update      | ~50 Hz     |
| UI Update (Serial)| ~5 Hz      |

This prevents UI or servo delays from affecting EMG acquisition.

---

### 4.3 Firmware Processing Pipeline

1. **ADC Sampling**
   - Fixed-rate sampling using `micros()`
   - 10-bit resolution

2. **Digital Filtering**
   - Spike rejection
   - Exponential Moving Average (EMA)

3. **Calibration & Normalization**
   - Baseline (rest) capture
   - Maximum voluntary contraction capture
   - Normalized EMG ∈ [0, 1]

4. **Control Mapping**
   - Normalized EMG mapped to servo angle
   - Constrained to safe mechanical limits

5. **Rate Limiting**
   - Limits angular velocity
   - Prevents abrupt or unsafe motion

---

## 5. Control State Machine

System behavior is governed by an explicit state machine:

### States
- **BOOT** – Initialization
- **IDLE** – No actuation, waiting for user input
- **CALIBRATION (Baseline / Max)** – EMG calibration sequence
- **ACTIVE** – Proportional assistive control
- **ERROR** – Fault-safe mode

### Transitions
- User-driven via buttons
- Automatic on fault detection

The servo is always driven to a safe rest position outside ACTIVE mode.

---

## 6. Mechanical Actuation Layer

### 6.1 Actuator
- High-torque hobby servo motor
- PWM-controlled
- Limited angular range for safety

### 6.2 Transmission Mechanism
- Four-bar linkage
- Single actuator driving:
  - Index finger
  - Middle finger
  - Ring finger

### 6.3 Design Intent
- Mimic natural finger curling trajectory
- Reduce number of actuators
- Minimize system mass and complexity

---

## 7. Safety Architecture

Safety is enforced at multiple layers:

### 7.1 Electrical Safety
- Low-voltage operation
- Isolation via EMG module
- External servo power supply

### 7.2 Software Safety
- Flat-signal detection
- Invalid calibration detection
- Servo saturation limits
- Automatic fallback to rest position

### 7.3 Mechanical Safety
- Limited servo travel
- Compliant mounting via straps
- Lightweight printed components

---

## 8. Design Trade-offs

| Aspect              | Decision Made                         |
|--------------------|----------------------------------------|
| Finger control     | Coupled (underactuated)                |
| EMG channels       | Single-channel                         |
| Control strategy   | Proportional, open-loop                |
| Actuator count     | One servo                              |
| Feedback sensing   | None (open-loop)                       |

These trade-offs prioritize reliability, wearability, and simplicity.

---

## 9. Extension Points

The architecture supports future upgrades, including:
- Multi-channel EMG
- Closed-loop force or position feedback
- Wireless microcontroller
- Machine-learning-based intent decoding
- Independent finger actuation

---

## 10. Summary

The system architecture integrates:
- Biological signal acquisition
- Deterministic embedded control
- Mechanically simplified actuation

The result is a lightweight, responsive, and extensible assistive exoskeleton architecture suitable for further research, iteration, and real-world experimentation.