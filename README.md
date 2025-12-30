# SilverHand – EMG-Controlled Assistive Hand Exoskeleton

SilverHand is a wearable, EMG-controlled assistive hand exoskeleton designed to provide proportional finger flexion assistance using surface electromyography and a mechanically simplified actuation strategy.
The system integrates analog EMG signal conditioning, real-time embedded control, and a single-actuator four-bar linkage to assist the index, middle, and ring fingers.

---

## System Overview

**Control modality**

* Single-channel surface EMG (sEMG) from forearm flexor muscles
* Proportional control based on EMG envelope magnitude

**Actuation**

* One high-torque servo motor
* Four-bar linkage driving three fingers simultaneously
* Safe, limited actuation range for assistive grasping

**Architecture**

```
Surface EMG → Analog Conditioning → Arduino Nano
            → Digital Filtering & Calibration
            → PWM Servo Control → Four-Bar Linkage → Finger Flexion
```

---

## Hardware Architecture

### Mechanical Subsystem

* **Mechanism:** Four-bar linkage optimized for finger curling trajectory
* **Actuated fingers:** Index, middle, ring (coupled motion)
* **Fabrication:** 3D-printed PLA components
* **Mounting:** Velcro straps for adjustability and comfort
* **Total device weight:** < 250 g

**Design goals**

* Reduce actuator count and mass
* Maintain natural curling motion
* Avoid mechanical interference within 0–45° servo rotation

---

### Actuator

* **Servo:** TowerPro MG996R
* **Operating voltage:** 4.8–7.2 V
* **Stall torque:** ~11 kg·cm @ 6 V
* **Control:** 50 Hz PWM
* **Utilized range:** ~0°–45° (mechanically constrained)

The servo provides sufficient torque for assistive grasping while remaining compact and widely available.

---

### EMG Acquisition & Signal Conditioning

The system uses an external analog EMG conditioning module that outputs a clean 0–5 V envelope signal suitable for direct ADC sampling.

**Signal chain**

1. Differential instrumentation amplification (high CMRR)
2. Precision full-wave rectification
3. Low-pass filtering / envelope detection
4. Level shifting to 0–5 V

**Typical observed levels**

* Rest: ~0.4–0.6 V
* Moderate contraction: ~1.5–2.2 V
* Strong contraction: ~3.0–3.8 V

---

## Electronics

* **Microcontroller:** Arduino Nano (ATmega328P)
* **ADC resolution:** 10-bit
* **Servo output:** Digital pin 3
* **Buttons:** 3× tactile buttons (UP / DOWN / SELECT)
* **Storage:** EEPROM for calibration data
* **Optional:** I2C OLED (reserved for future UI)

---

## Firmware Design

The firmware is structured as a deterministic, state-machine-driven control system rather than a simple threshold trigger.

### Key Features

#### 1. Fixed-Rate EMG Sampling

* ~1 kHz sampling using `micros()`
* Sampling decoupled from UI and servo updates

#### 2. Digital Signal Processing

* Spike rejection
* Exponential Moving Average (EMA) filtering
* Stable EMG envelope for proportional control

#### 3. Calibration & Normalization

Two-phase calibration:

1. **Baseline** (resting muscle)
2. **Maximum voluntary contraction**

* Stored in EEPROM with checksum validation
* All EMG values normalized to a 0–1 activation scale

#### 4. Servo Mapping & Safety

* Normalized EMG mapped to a constrained servo angle range
* Rate limiting to avoid sudden motion
* Automatic return to safe position on fault

#### 5. Fault Detection

* Flat-signal detection
* Invalid calibration handling
* Signal saturation checks

#### 6. Text-Based User Interface

* Serial Monitor UI
* Button-driven menu system
* Modes:

  * BOOT
  * IDLE
  * CALIBRATION (Baseline / Max)
  * ACTIVE
  * ERROR

---

## Firmware State Machine

```
BOOT → IDLE
        ↓
   CALIBRATION
        ↓
     ACTIVE
        ↓
      ERROR (on fault)
```

Each state enforces safe actuator behavior and explicit user intent.

---

## Repository Structure (Suggested)

```
/firmware
  └── emg_servo_controller.ino

/mechanical
  ├── linkage_cad/
  ├── enclosure_cad/
  └── stl/

/electronics
  ├── emg_module_schematic.pdf
  └── wiring_diagram.png

/docs
  ├── system_architecture.md
  └── calibration_guide.md
```

---

## Build & Setup

### Hardware Setup

1. Mount linkage on dorsal side of hand
2. Attach finger caps to index, middle, ring fingers
3. Place EMG electrodes over forearm flexor muscles
4. Connect EMG output to Arduino A0
5. Power servo from an external 5–6 V supply

### Firmware

1. Open firmware in Arduino IDE
2. Select **Arduino Nano / ATmega328P**
3. Upload sketch
4. Open Serial Monitor (115200 baud)
5. Run calibration sequence before active use

---

## Performance Characteristics

* **End-to-end latency:** < 50 ms
* **Motion type:** Proportional assistive flexion
* **Grasp capability:** Light objects (cups, pens, handles)
* **Noise:** Minimal mechanical noise from linkage

---

## Limitations

* Single actuator → no independent finger control
* Surface EMG requires per-user calibration
* No force or position feedback
* Open-loop actuation

These are deliberate trade-offs to maintain mechanical simplicity and low system mass.

---

## Future Extensions

* Multi-channel EMG for finger-level control
* Closed-loop force or position sensing
* Wireless MCU (ESP32) + battery operation
* Adaptive or ML-based intent recognition
* On-device display UI
* Modular finger supports for different hand sizes

---

## License

This project is licensed under the CERN Open Hardware License v2 – Strongly Reciprocal (CERN-OHL-S).

---
