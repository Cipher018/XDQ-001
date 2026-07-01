# XDQ-001

## Introduction

## What is XDQ-001?

The **XDQ-001**, or **Experimental Drone Quadcopter Model 001**, is a flight platform with an original design. Its experimental objective is to achieve a FPV quadcopter capable of both acrobatic flight and cinematic filming. Furthermore, the XDQ-001 platform serves as a cutting-edge testbed for implementing highly optimized low-level control and autonomous navigation algorithms.

## Whats New?

The **XDQ-001** represents a massive generational leap over its predecessor (XDF-001), integrating the following hardware and firmware innovations (Version V001):

1. **Hybrid Cooperative Interrupt-Based Architecture (No RTOS):**
   - **1000Hz Critical Loop:** Hardware Timer ISR dedicated exclusively to attitude (AHRS), PID calculations, and motor control.
   - **50Hz Kinematic Loop:** Background loop for Kalman filtering, waypoint navigation, and geometric estimation.
2. **Dual-Layer Navigation System:**
   - **Layer 1 (Attitude):** Optimized Mahony AHRS filter running at 1000Hz.
   - **Layer 2 (Kinematics):** Three-Dimensional Error State Kalman Filter (3D ESKF) at 50Hz, fusing accelerometer, GPS, and barometer (BMP280) measurements.
3. **CADI_A Adaptive Navigation System:**
   - Advanced algorithms for L1 navigation (adaptive waypoints) and Orbit (loiter).
   - **Stochastic Wind Estimator (Pitot-less):** Calculates and compensates for crosswind in real-time without requiring a pitot tube, based on geometric discrepancy during turns.
4. **Advanced Flight Control (FlightPID):** Isolated bandwidth PIDs (LPF_PT1 filters on P and D terms).
5. **Dynamic Safety Guards:**
   - *Dynamic Stall Guard:* Dynamic stall prevention based on bank angle.
   - *G-Limiter:* Pilot control limitation when the safe structural G-load (2.0G - 3.5G) is exceeded.
   - *Failsafe RTL:* Safe automatic Return-To-Launch on link loss.
6. **Dedicated Hardware:** MCU STM32F4 (with FPU), dedicated ICM-42688 IMU, MMC5983MA external magnetometer, and SX1280 long-range modem.
7. **Secure Communications & Telemetry:** Frequency Hopping Spread Spectrum (FHSS), logical dual-pipe, and strong cross-block encryption (XXTEA) with integrated CRC16.
8. **Uninterrupted Blackbox Logging:** Lock-free Ping-Pong buffers linked via DMA to the W25Q32JVSS Flash memory.

## Objectives

The primary objective is to develop an exceptional FPV quadcopter capable of both extreme acrobatic flight and smooth cinematic tracking.

The secondary objectives are:
- Provide an ultra-fast, predictable, non-RTOS software framework.
- Consolidate the capabilities of the CADI autonomous ecosystem (Waypoints, Orbits, auto RTL).
- Implement and refine stochastic estimation systems (ESKF and Wind Estimator).
- Establish an impenetrable telemetry system (robust encryption and FHSS).

## Architecture

As previously mentioned, the project is based on the STM32F4 architecture utilizing a hybrid cooperative interrupt-based system. You can consult the documentation section for details regarding the PCB, algorithms, or flowcharts.

## Hardware

The XDQ-001 uses a custom Flight Control Board (FCB) built around the **STM32F4** microcontroller. Key sensors and components:

- **IMU (Gyro/Accel):** ICM‑42688‑P
- **Magnetometer:** MMC5983MA
- **Barometer:** BMP280
- **Radio Link:** SX‑1280 (FHSS Support)
- **Storage:** W25Q32JVSS Flash Memory
- **Camera:** Caddx Baby Ratel 2 (optional)

A full Bill of Materials (BOM) is still being defined.

## Software

- Development is currently performed using the **Arduino IDE** (with the STM32 core) but migration to a more advanced toolchain is planned.
- No ground control station (GCS) software exists yet; a future GCS will communicate via the SX‑1280 radio.

## Compilation & Installation

The firmware follows the standard Arduino IDE project layout:

1. Open the `Firmware/` folder and load `XDQ-001F_V001.ino`.
2. Select the appropriate **STM32F4** board definition.
3. Ensure FPU support is enabled if required.
4. Verify / compile the sketch (`Ctrl+R`) and upload using a compatible programmer (e.g., ST-Link V2) or USB DFU bootloader.

## Usage

1. Power the FCB and connect the radio module.
2. Power the drone and perform a basic motor test (ensure propellers are removed).
3. When the GCS becomes available, pair it with the SX‑1280 radio and start telemetry.

## Configuration

At present the firmware has no runtime‑configurable parameters. Future releases will expose PID and filter tuning via the GCS.

## Contributing

Contributions are welcome under an **MIT license**. Please:

- Fork the repository and create a feature branch.
- Follow the existing code style (Arduino‑C++ with descriptive comments).
- Ensure any additions maintain the non-blocking philosophy of the 1000Hz critical loop.
- Open a Pull Request with a clear description of changes.

## License

This project is licensed under the **MIT License**. See the `LICENSE` file for details.

## Documentation

It is highly recommended to review the comprehensive documentation to understand the mathematical and algorithmic details:

- 📄 **[Firmware Reference Manual (Markdown)](Documentation/firmware_reference.md)**
- 📕 **[Firmware Reference Manual (PDF)](Documentation/firmware_reference.pdf)**
- 📐 [Architecture Diagram](Documentation/architecture.md) (Pending migration)
- 🔌 [Pinout & Hardware](Documentation/pinout.md) (Pending migration)
- 📡 [Protocols](Documentation/protocol.md) (Pending migration)

## Project Structure

The repository is organized as follows:

- **Firmware/** – Arduino sketch source files (`*.ino`, drivers, control algorithms, telemetry).
- **Documentation/** – Design documents, architecture diagrams, pin‑out tables, protocol specifications, and other reference material.
- **CAD/** – 3D models and mechanical drawings for printable parts.
- **PCB/** – PCB design files, gerbers, and component placement.
- **References/** – Datasheets, application notes, and external resources used during development.
- **README.md** – This high‑level overview.
- **LICENSE** – MIT license file.
