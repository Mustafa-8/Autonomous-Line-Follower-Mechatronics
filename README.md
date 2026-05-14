# Autonomous Line-Follower Robot

A small-scale autonomous line-following robot built as a final project for the **Applied Automatization** course at the Faculty of Engineering, University of Debrecen. Developed as a scaled model of an industrial Automated Guided Vehicle (AGV).

The robot uses a Proportional-Derivative (PD) control algorithm wrapped in a four-state supervisory state machine to track an 18 mm black line on a light-coloured surface, with deterministic recovery logic for line-loss events.

---

## Features

- **Custom 3D-printed chassis** designed in Autodesk Fusion 360 with slotted mounting features for assembly-time adjustability
- **Five-element TCRT5000 infrared sensor array** with weighted-average error formulation
- **PD control loop** with experimentally tuned gains (Kp = 12, Kd = 14)
- **Four-state supervisory state machine** (STRAIGHT, CURVE, SHARP, LOST) adapting baseline speed to track geometry
- **Memory-based line-loss recovery** that pivots the robot back toward the last-known line position
- **Software-only hardware calibration** for motor speed asymmetry and direction inversion
- **Reserved encoder lines** on Arduino D9–D12 for future closed-loop velocity control via Pin Change Interrupts

---

## Hardware

| Subsystem | Component | Notes |
|---|---|---|
| Microcontroller | Arduino Nano (ATmega328P, Type-C) | Main control unit |
| Motor driver | TB6612FNG | Dual H-bridge, PWM speed + direction |
| Drive motors | 2 × N20 micro metal gear motors | 6 V, 500 RPM, integrated encoders |
| Wheels | 2 × 42 mm rubber-tyred | Fitted to 3 mm D-shafts |
| Front support | Steel ball castor | Adjustable height via 6-nut stack |
| Sensors | 5 × TCRT5000-M modules | Digital output, onboard sensitivity trimmer |
| Battery | 2 × 18650 Li-Ion in series | 7.4 V nominal |
| Regulator | LM2596S buck converter | Calibrated to 5.0 V output |
| Power switch | Panel-mount SPST | Master on/off |
| Chassis | 3D-printed, 155 mm × 115 mm × 2.7 mm | Custom stepped polygon |

---

## Wiring (Arduino Nano ↔ TB6612FNG)

| Arduino Pin | Driver Pin | Function |
|---|---|---|
| D6 | STBY | Driver standby (HIGH = enabled) |
| D3 (PWM) | PWMA | Right motor speed |
| D7 | AIN1 | Right motor direction |
| D8 | AIN2 | Right motor direction |
| D5 (PWM) | PWMB | Left motor speed |
| D4 | BIN1 | Left motor direction |
| D2 | BIN2 | Left motor direction |
| A0–A4 | (sensors) | 5 × digital sensor inputs |
| D9–D12 | (encoders) | Reserved for future PCINT |

The TB6612FNG receives 7.4 V on VM (motor supply) and 5.0 V on VCC (logic supply) from the LM2596S buck converter. All grounds are common.

---

## How to compile and upload

1. Install the **Arduino IDE** (https://www.arduino.cc/en/software).
2. Connect the Arduino Nano via USB-C.
3. In the Arduino IDE, select:
   - **Board:** Arduino Nano
   - **Processor:** ATmega328P (Old Bootloader)
   - **Port:** the appropriate COM port for your system
4. Open `firmware/line_follower.ino`.
5. Click **Upload**.

The firmware enters a 2-second startup delay after power-on, allowing the operator to place the robot on the track before motion begins.

---

## Algorithm overview

Each iteration of the main control loop performs the following steps:

1. **Sample** the five digital sensor outputs.
2. **Compute** the weighted-average error signal e ∈ [−4, +4].
3. **Classify** the operating regime (STRAIGHT, CURVE, SHARP, or LOST) based on |e|.
4. **Look up** the baseline wheel commands (b_L, b_R) for that regime.
5. **Compute** the PD correction: u = Kp·e + Kd·(e − e_prev).
6. **Apply** the correction differentially: left = b_L + u, right = b_R − u.
7. **Saturate** to the PWM range and write to the motor driver.

If no sensor detects the line, the controller substitutes a saturated recovery error in the direction of the last-known line position, producing an in-place rotational search.

---

## Project status

- ✅ Mechanical assembly complete
- ✅ Electrical wiring complete
- ✅ Firmware complete and tuned
- ✅ Validation testing complete (multiple successful full-lap runs)
- 📄 Technical report complete (see `docs/`)

### Known limitations

The sensor working distance is approximately 15 mm above the floor due to a geometric constraint imposed by the fixed under-chassis battery mounting, rather than the TCRT5000's nominal 2–4 mm working window. This causes three observed effects: inconsistent detection of sharp 90° turns, sensitivity to low ambient lighting, and a small dead zone around the line centreline on gradual curves. See the technical report (Section VII) for full analysis.

### Future work

- Resolve the sensor-height limitation via chassis redesign or thinner battery form factor
- Implement closed-loop velocity control using the installed encoders via Pin Change Interrupts on Port B
- Extend the state machine to handle intersections and end-of-track markers

---

## Repository structure

```
Autonomous-Line-Follower-Mechatronics/
├── README.md
├── LICENSE
├── firmware/
│   └── line_follower.ino
├── docs/
│   └── Final_Report.pdf
└── hardware/
    └── (CAD files, schematics, photos)
```

---

## Documentation

The complete technical report is available in the `docs/` folder and includes:

- Mathematical modeling and PD control derivation
- Mechanical design with chassis geometry and slotted mounting strategy
- Electrical schematic and bill of materials
- Control system architecture and state machine logic
- Software implementation details
- Testing and validation results
- Identified limitations and future work

---

## Team

| Role | Member |
|---|---|
| Team Leader / Lead Programmer | Mustafa Ali Mohsin Albusultan |
| Mechanical Build Engineer | Muhammad Salihu |
| Electronics & Power Specialist | Ahsan Muhammad |
| Systems Testing & Documentation | Abdul Fathah |

**Course:** Applied Automatization
**Institution:** Faculty of Engineering, University of Debrecen
**Year:** 2026

---

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

Built as part of the Applied Automatization course curriculum. Thanks to the course supervisor and the Faculty of Engineering for project guidance and laboratory facilities.
