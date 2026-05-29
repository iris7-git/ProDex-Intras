# PivotPoint — Real-Time ACL Injury Prevention System

> A smart wearable brace that detects dangerous knee rotation in real-time and delivers instant haptic warnings to prevent ACL injuries before they happen.

---

## The Problem

ACL injuries occur when the knee twists too far, too fast — typically during landings or pivots. The critical issue: **there is no warning**. Standard braces provide mechanical support but cannot detect the hidden rotational torque that causes ligament rupture, leaving athletes moving in high-risk positions with no feedback.

---

## Solution Overview

PivotPoint combines a **mechanical support frame** and an **IoT monitoring system** into a single wearable device.

### Part 1 — Mechanical Frame

An external skeleton that stabilizes the knee joint without restricting natural athletic movement.

| Component | Function |
|---|---|
| **Dual Tapered Cuffs** (Thigh + Calf) | Conform to leg geometry; prevent slipping during dynamic movement |
| **Aluminum Lateral Rods** | Rigid side-rails anchored to thigh and calf cuffs via bolted slots |
| **Sliding-Slot Kinematic Hinge** | Oval vertical slot on calf rod allows free knee flexion while resisting rotational torque |
| **Revolute Lap-Joint at Knee** | Mimics the knee's primary axis of rotation; metal stiffness resists twisting forces |
| **D-Ring Velcro Strap System** | High-compression lock ensuring the frame stays synced with bone movement |

The rods act as **load-sharing partners** — absorbing a portion of rotational torque that would otherwise be borne entirely by the ACL.

### Part 2 — IoT Monitoring System

| Component | Role |
|---|---|
| **ESP32 Microcontroller** | Central processing unit; pulls sensor data and drives outputs |
| **2× MPU-6050 IMU Sensors** | One in thigh cuff, one in calf cuff; track femur vs. tibia orientation |
| **AD0 Address Offset Wiring** | Allows two identical I2C sensors to share one ESP32 bus |
| **ERM Haptic Vibration Motor** | Embedded against skin; fires when a danger threshold is crossed |
| **Bluetooth Module** | Streams movement data wirelessly for logging and analysis |

**Processing pipeline:**
1. ESP32 reads yaw angles from both MPU-6050s at high frequency
2. Computes **Twist Angle** = difference between thigh and calf rotation (differential yaw)
3. If Twist Angle exceeds the **Red Zone threshold (15°–20° internal rotation)** → PWM signal triggers haptic motor
4. Athlete feels sharp vibration → corrects form before ACL reaches failure point

---

## Key Features

- **Real-Time Differential Processing** — Twist angle computed on-device; no cloud dependency
- **Haptic Biofeedback Alert** — Instant tactile warning at the danger threshold
- **Kinematic Sliding-Slot Hinge** — Full knee flexion preserved while rotational torque is resisted
- **Dual-Node IMU Array** — Independent tracking of femur and tibia segments
- **Load-Sharing Frame** — Physically absorbs mechanical stress to reduce ligament load
- **Bluetooth Data Logging** — Transmits biomechanical data to smartphone for coach/doctor review

---

## Hardware & Cost

| Category | Component | Cost (Est.) |
|---|---|---|
| Microcontroller & Sensors | ESP32 + 2× MPU-6050 | ₹800 |
| Haptic Feedback | ERM Coin Vibration Motor + MOSFET Driver | ₹150 |
| Power Supply | 3.7V Li-ion Battery + Slide Switch | ₹100 |
| Wearable Base | Compression Knee Sleeve | ₹450 |
| Structural Frame | PETG / Nylon / CF-PLA Filament (3D Printed) | ₹300 |
| Fasteners | M4 Shoulder Bolts, Nyloc Nuts, M2/M3 Nylon Screws + Jumper Wires | ₹150 |
| **Total** | | **~₹1,950** |

---

## Tech Stack

- **Firmware:** ESP32 (Arduino / ESP-IDF)
- **Sensors:** MPU-6050 (I2C, AD0 address differentiation)
- **Communication:** Bluetooth (data logging to smartphone)
- **Mechanical:** 3D-printed PETG/Nylon cuffs, aluminum rods, lap-joint hinge
- **Feedback:** ERM haptic motor via PWM

---

## References

- [Mechanism of Non-Contact ACL Injuries](https://pmc.ncbi.nlm.nih.gov/articles/PMC8858885/)
- [Tibial Internal Rotation and ACL Risk](https://pmc.ncbi.nlm.nih.gov/articles/PMC10768481/)
- [Knee Flexion Angles and ACL Injuries](https://www.medrxiv.org/content/10.1101/2025.10.20.25338317v1.full.pdf)
- [Real-Time Monitoring System for ACL Injury Prevention](https://www.researchgate.net/publication/385367697_Design_and_Development_of_a_Real-time_Monitoring_System_for_ACL_Injury_Prevention)
- [ESP32 Programming Guide](https://www.raypcb.com/esp32-programming-circuit/)

---

## Future Work

- **ML-Based Predictive Analytics** — Use Bluetooth-logged biomechanical history to build a personalized movement baseline per athlete, enabling prediction of fatigue-driven injury risk *before* the knee enters a danger zone
- Cloud dashboard for coaches and physiotherapists
- Multi-joint support (ankle integration)

---

## Team

| Name |
|---|
| Harshil Mishra |
| Shreesh Tripathi |
| Bhargavi Deo |
| Nandan Krishna S |
| Kamalnath |

*Developed as part of IntraProdex, IIT Kharagpur.*
