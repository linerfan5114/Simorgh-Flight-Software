# Simorgh Pro Flight Software

Advanced CubeSat flight software with EKF+LQR attitude control, SGP4 orbit propagation, CFDP file transfer, AES-256-GCM encryption, TMR fault tolerance, and real-time scheduler.

## Version
3.0.0

## Features

- **ADCS**: 7-state EKF + LQR controller
- **Orbit**: SGP4 propagator with TLE
- **Power**: Li-Ion battery model + MPPT
- **Thermal**: 10-node thermal network
- **Comms**: CCSDS + AX.25 + CFDP + AES-256-GCM
- **Fault Protection**: TMR voter + EDAC + Watchdog
- **RTOS**: Priority-based scheduler @ 100Hz
- **Mission Planner**: Automated target scheduling

## Build

```bash
make