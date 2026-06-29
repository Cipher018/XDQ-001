# XDQ-001

## Introduction

## What is XDQ-001?

The **XDQ-001**, or **Experimental Drone Quadcopter Model 001**, is a flight platform with original design. Its experimental objective is to achieve a FPV quadcopter capable of both acrobatic flight and cinematic filming. Furthermore, the XDQ-001 platform seeks to implement different filters from different sources for PID control and autonomous navigation by means of external or internal software.

## Whats New?

The **XDQ-001** With respect to my previous project, the aircraft designated as **XDF-001** features the following improvements and modifications
1. Microcontroller changed to an STM32F4 for improved speed and processing of control algorithms
2. A BMP-280 barometer was added to obtain accurate altitude data.
3. The MPU9250 IMU was replaced by a dedicated IMU and external magnetometer
4. Introduction of new control algorithms such as Mahony filters and 3-dimensional Kalman filters
5. Improved PID control
6. Dedicated and compact FCB to reduce chassis size
7. Camera changed to a Caddx Baby Ratel 2 for better video quality
8. 3D printable parts without the need for additional tools
9. FLASH memory for logs

## Objectives

The primary objective is to develop a FPV quadcopter capable of both acrobatic flight and cinematic filming.

The secondary objectives are:

- Obtain an autocalibration system through PID and motor tests for 4 motors.
- Implementation of new control algorithms such as Mahony filters and 3-dimensional Kalman filters
- Implement autonomous functions such as waypoint navigation and orbiting
- Improvement of PID control
- Recording logs and telemetry data to flash memory

## Architecture

As previously mentioned, the project is based on the STM32F4 architecture. You can consult the documentation section for details regarding the PCB, algorithms, or flowcharts.
## Hardware

The XDQ-001 uses a custom Flight Control Board (FCB) built around the **STM32F4** microcontroller. Key sensors and components:

- **IMU**: ICM‑42688‑P (gyroscope + accelerometer)
- **Magnetometer**: MMC5983MA
- **Barometer**: BMP280
- **Radio**: SX‑1280 (NRF24‑compatible) for telemetry and control
- **Camera**: Caddx Baby Ratel 2 (optional)
- **Flash storage** for logs

A full Bill of Materials (BOM) is still being defined.

## Software

- Development is performed with the **Arduino IDE** (currently) but migration to a more advanced toolchain is planned.
- No ground control station (GCS) software exists yet; a future GCS will communicate via the SX‑1280 radio.

## Compilation & Installation

The firmware follows the standard Arduino IDE project layout:

1. Open the `XDQ-001` folder in Arduino IDE.
2. Select the appropriate **STM32F4** board definition.
3. Verify / compile the sketch (`Ctrl+R`).
4. Upload to the FCB using a compatible programmer (e.g., ST‑Link) or USB bootloader.

The source code is organized under `Firmware/` with separate `.ino` files for sensor drivers and control loops. Adjust the `src/` folder if you adopt another build system.

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
- Open a Pull Request with a clear description of changes.

See the `Documentation/` folder for detailed architecture, pin‑out diagrams, and protocol specifications.

## License

This project is licensed under the **MIT License**. See the `LICENSE` file for details.

## Documentation

- [Architecture](Documentation/architecture.md)
- [Pinout](Documentation/pinout.md)
- [Protocol](Documentation/protocol.md)
- Additional reference material is located in the `Documentation/` directory.
## Project Structure

The repository is organized as follows:

- **Firmware/** – Arduino sketch source files (`*.ino`, drivers, control algorithms, telemetry).
- **Documentation/** – Design documents, architecture diagrams, pin‑out tables, protocol specifications, and other reference material.
- **CAD/** – 3D models and mechanical drawings for printable parts.
- **PCB/** – PCB design files, gerbers, and component placement.
- **References/** – Datasheets, application notes, and external resources used during development.
- **README.md** – This high‑level overview.
- **LICENSE** – MIT license file.
