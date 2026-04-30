# Simorgh Pro – CubeSat Flight Software Framework

**A modular, high-integrity flight software suite for LEO microsatellite missions**

![Version](https://img.shields.io/badge/version-3.0.0-blue)
![Language](https://img.shields.io/badge/language-C11-555555)
![License](https://img.shields.io/badge/license-MIT-green)
![Build](https://img.shields.io/badge/build-gcc%20-O3-brightgreen)

---

## 📡 Overview

**Simorgh Pro** is a self-contained, bare-metal-style flight software framework designed for 3U/6U CubeSats operating in Low Earth Orbit (LEO). It implements a complete Guidance, Navigation, and Control (GNC) pipeline, power-aware state management, CCSDS-compliant telemetry, AX.25 radio communication, a lightweight flash file system, and radiation-hardening-by-software techniques – all within a deterministic, single-binary architecture suitable for resource-constrained embedded processors.

The codebase is structured for both **hardware-in-the-loop testing** and **full-mission desktop simulation**, running accelerated orbital scenarios with configurable durations.

---

## 🚀 Key Capabilities

| Subsystem | Implementation |
|-----------|----------------|
| **ADCS** | EKF-based attitude estimation + LQR optimal controller + reaction wheel & magnetorquer actuation |
| **Orbit Propagation** | SGP4 analytical propagator with TLE ingestion |
| **Power (EPS)** | Coupled solar panel and battery model with load-shedding logic |
| **Thermal** | Lumped-node thermal network with autonomous heater control |
| **Communications** | AX.25 UI frames, CCSDS Space Packet Protocol, CFDP file transfer stubs |
| **Fault Protection** | Multi-tier watchdog, TMR voting, memory scrubbing, EDAC, safe mode escalation |
| **Mission Planning** | Target scheduling with power gating and payload sequencing |
| **Data Handling** | Simple block-based file system (SD/Flash emulation), science data buffering |
| **Crypto** | AES-256-GCM (lightweight reference implementation), CRC-16/32 |

---

## 📂 Project Structure

```
simorgh-pro/
├── src/
│   └── simorgh_pro.c        # Full flight software (single compilation unit)
│   └── simorgh_pro.h        # Header with all type definitions, constants, and prototypes
├── config/
│   ├── simorgh.conf         # Persistent configuration (auto-generated on first run)
│   └── tle.txt              # Two-Line Element set for initial orbit
├── Makefile                 # Build automation
└── README.md
```

> **Architecture Note:** The entire flight software is contained in two files (`simorgh_pro.c` and `simorgh_pro.h`). This mirrors the monolithic binary approach common in embedded flight computers, where dynamic linking is unavailable and a single executable image is flashed to redundant boot banks.

---

## ⚙️ Build & Run

### Prerequisites

- **GCC** (version 7+ recommended)
- **POSIX-compatible environment** (Linux, macOS, WSL)
- **Math library** (`libm`)

### Quick Start

```bash
# Build the simulation binary
make

# Run with default 86400-second (1-day) simulation
make run

# Run with custom duration (e.g., 3 orbits ≈ 17010 seconds)
./simorgh_pro 17010
```

The program executes an accelerated discrete-time simulation loop (100 Hz tick resolution) and prints periodic status updates to `stdout`. A final mission summary is displayed on exit.

### Build Options

| Target | Description |
|--------|-------------|
| `make` | Compile with `-O3 -march=native` |
| `make run` | Build and execute with default duration |
| `make clean` | Remove binary |
| `make distclean` | Remove binary and generated config files |

---

## 🧠 Software Architecture

### Execution Flow (`main`)

1. **`satellite_init()`** – Load persistent config, parse TLE, initialize all subsystems, print banner.
2. **Simulation Loop** (10-second steps, 10 ms ticks):
   - `sensors_update()` – Propagate orbit via SGP4, generate noisy sensor measurements.
   - `gps_update()` / `star_tracker_update()` – Populate navigation sensor structs.
   - `deployment_update()` – Sequence antenna/panel/magnetometer deployment.
   - `state_machine()` – Transition between operational states (Init → Detumble → Sun Acquisition → Nominal).
   - `eps_update()` – Manage power budget, enable/disable loads.
   - `thermal_update()` – Run thermal network step, toggle heaters.
   - `adcs_control()` – Run EKF predict/update, compute LQR torque, saturate actuators.
   - `ccsds_pack_tm()` – Build CCSDS telemetry packet with full housekeeping.
   - `radio_recv()` / `command_process()` – Handle ground commands.
   - `radio_beacon()` – Transmit periodic status beacon.
   - `fs_save_tm()` – Write telemetry to flash file system.
   - `mission_planner_update()` – Trigger payload acquisitions on schedule.
   - Fault monitoring (`watchdog_kick`, `memory_scrubber`, `tmr_check`).
3. **`satellite_shutdown()`** – Print mission statistics, save configuration.

### State Machine

```
┌─────────┐    deploy complete     ┌───────────┐
│  INIT   │ ──────▶──────────────▶ │ DETUMBLE  │
└─────────┘                        └─────┬─────┘
                                         │ ω < 0.015 rad/s
                                         ▼
                               ┌──────────────────┐
                               │ SUN ACQUISITION  │
                               └────────┬─────────┘
                                        │ sun_vec[1] < -0.985
                                        ▼
                        ┌─────────────────────────────┐
                        │ NOMINAL EARTH POINTING      │
                        └─────────────┬───────────────┘
                                      │ Vbat < safe_voltage
                                      ▼
                               ┌─────────────┐
                               │ SAFE MODE   │
                               └──────┬──────┘
                                      │ Vbat < crit_voltage
                                      ▼
                        ┌────────────────────────┐
                        │ EMERGENCY SHUTDOWN     │
                        └────────────────────────┘
```

### Attitude Determination & Control

- **Estimator:** 7-state Extended Kalman Filter (quaternion + gyro bias) with process/measurement noise tuning.
- **Sensors:** Gyroscope, magnetometer, coarse sun sensor, star tracker (validity-gated).
- **Controller:** Infinite-horizon LQR with gain matrix `K ∈ ℝ³ˣ⁴`.
- **Actuators:** 4 reaction wheels (torque-limited, desaturation logic) + 3-axis magnetorquers for momentum dumping.

### Fault Detection, Isolation & Recovery (FDIR)

| Mechanism | Description |
|-----------|-------------|
| **Watchdog** | Hardware-alike software watchdog with counter-based timeout; triggers full re-init |
| **TMR Voting** | Triple-modular redundancy on critical state outputs (3-computer emulation) |
| **Memory Scrubbing** | Periodic refresh of critical variables (SEU mitigation) |
| **EDAC** | Hamming-code SECDED for memory words (detection stub) |
| **Fault Escalation** | Over-temperature, over-current, wheel overspeed → safe mode → emergency |

---

## 📡 Communication Protocols

### Downlink (TM)

- **CCSDS Space Packet** (Packet ID `0x18A0`, secondary header with mission timestamp).
- **AX.25** HDLC framing with UI (unnumbered information) control field.
- Beacon: ASCII status string at configurable interval.
- Full telemetry struct (battery, temps, attitude, GPS, fuel) CRC-16 protected.

### Uplink (TC)

- **Command Packet** with sync word `0xEB90`, XOR checksum, 64-byte payload.
- Supported ops: mode switch, time sync, payload control, telemetry request, file download, maneuver burn, software reset.

### File Transfer

- **CFDP** (Class 2) stub: file metadata announcement via AX.25, data recovery through SD block interface.

---

## 📊 Simulation Fidelity

| Component | Model |
|-----------|-------|
| Orbit | SGP4 analytical propagator with J2 perturbation |
| Solar panels | Angle-dependent efficiency with temperature derating |
| Battery | Coulomb-counting voltage model with temperature compensation |
| Sensors | Gaussian noise + bias + eclipse-based validity |
| Thermal | 10-node lumped thermal network with internal heating |
| Actuators | Torque-limited, speed-damped, with power consumption |

---

## 🔧 Configuration

The `Config` structure (persisted to `config/simorgh.conf`) allows runtime tuning of:

- Ground station callsign
- Beacon and telemetry intervals
- Voltage and temperature safety thresholds
- ADCS gains (PID + rate damping)
- Subsystem enable flags (GPS, star tracker, payload, crypto, TMR)

Default values are hardcoded as failsafe constants and written on first boot if no config file exists.

---

## 📈 Mission Statistics

On shutdown, the software prints a comprehensive mission summary:

```
========================================
         MISSION COMPLETE
========================================
Uptime: 86400 sec (15.2 orbits)
Boots: 1 | Safe Modes: 0 | Emergencies: 0
Min Battery: 22.80V | Max MCU: 42.3°C
Radio: Sent=1440 Recv=0 Drop=0
Energy: Gen=1560Wh Con=1240Wh
Fuel: 2.500kg | Delta-V: 0.000m/s
========================================
```

---

## 🛰️ Heritage & Applicability

Simorgh Pro is designed as a reference architecture for academic CubeSat teams, flight software bootcamps, and rapid prototyping of ADCS algorithms. Its monolithic structure, while unusual for desktop software, reflects the constraints of radiation-hardened microcontrollers (e.g., MSP430, ARM Cortex-R, LEON3) where dynamic memory allocation and multi-file linking are avoided for reliability.



*"Per aspera ad astra" – Through hardship to the stars.* 🌟
