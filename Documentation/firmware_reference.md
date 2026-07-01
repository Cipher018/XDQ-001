# Firmware Reference Manual - XDQ-001
**Document Version:** 1.0  
**Firmware Version:** V001  
**Platform:** STM32F4 (with FPU)  

---

## Table of Contents
1. [System Architecture](#1-system-architecture)
2. [Dual-Layer Navigation System](#2-dual-layer-navigation-system)
3. [Flight Control & Safety](#3-flight-control--safety)
4. [Adaptive Navigation & Wind Estimation](#4-adaptive-navigation--wind-estimation)
5. [Secure Communications & Telemetry](#5-secure-communications--telemetry)
6. [Blackbox Logging](#6-blackbox-logging)

---

## 1. System Architecture

The XDQ-001 firmware implements a hybrid cooperative interrupt-based architecture (without RTOS) designed to optimize CPU cycles and guarantee strict timings in critical loops.

*   **Critical Attitude Loop (1000Hz):** Executed via a Hardware Timer Interrupt (`Timer_ISR`). This loop handles IMU reading, the AHRS filter, PID calculations, and PWM mixer updates for the motors.
*   **Kinematic and Cooperative Loop (50Hz):** Executed in the main block (`loop`). It handles less time-critical tasks such as reading the GPS, Magnetometer, and Barometer, executing the Kalman Filter, and waypoint navigation.

---

## 2. Dual-Layer Navigation System

One of the main technological innovations of the XDQ-001 is its dual approach to estimating attitude and kinematics.

### 2.1. Layer 1: Mahony AHRS Filter (Attitude)
For high-frequency orientation estimation, the firmware utilizes an optimized **Mahony AHRS** filter, processed in the 1000Hz loop.
*   Integrates raw high-speed readings from the ICM-42688 (Gyroscope and Accelerometer).
*   Maintains short-term stability and calculates quaternions that are converted to Euler angles (Roll, Pitch, Yaw), which are essential for acrobatic flight PIDs.

### 2.2. Layer 2: 3D Error State Kalman Filter (Kinematics)
For autonomous navigation, an Error State Kalman Filter (ESKF) processed at 50Hz fuses data from multiple sensors:
*   **System Model:** Maintains the state vector for 3D position and 3D velocity (N/E/D Position, N/E/D Velocity).
*   **Prediction:** Integrates accelerations transformed to the Earth's geographic frame using the AHRS quaternions.
*   **Update:** Uses latitude, longitude, altitude (fused with Barometer), and velocities provided by the GPS.

---

## 3. Flight Control & Safety

Stabilized control is achieved through an isolated controller class (`FlightPID`), complemented by dynamic aerodynamic safeguards.

### 3.1. Isolated Bandwidth PIDs
The PID loops implement PT1 low-pass filters (`LPF_PT1`) on the Proportional and Derivative terms, allowing more aggressive gains while rejecting electrical noise and mechanical vibrations induced by the frame.

### 3.2. Dynamic Safeguards
*   **Dynamic Stall Guard:** Continuously adjusts the stall speed threshold based on the bank angle. If the aircraft's speed drops below the safe threshold, the system autonomously injects negative pitch (nose down) and raises the throttle.
*   **G-Limiter:** Monitors the total G-load measured by the accelerometers and progressively attenuates pilot setpoints when detecting between 2.0 and 3.5 Gs to protect the structural frame.

---

## 4. Adaptive Navigation & Wind Estimation

The firmware introduces advanced autonomous capabilities from the CADI_A ecosystem.

### 4.1. Stochastic Wind Estimator (Pitot-less)
The XDQ-001 calculates the 2D wind vector by comparing the ground velocity vector (from GPS) with the directional attitude vector (nose direction). By applying a low-pass filter and utilizing averages during turns, the drone can determine wind direction and speed without additional hardware.

### 4.2. Guidance Algorithms (L1 & Orbit)
*   **Adaptive L1 Navigation:** Uses a lateral acceleration (L1) algorithm to smoothly generate roll corrections towards the next waypoint, integrating the crosswind compensation calculated by the wind estimator.
*   **TECS Control (Energy):** Employs a simplified variant of the Total Energy Control System (TECS), managing pitch (potential energy) and throttle (kinetic energy) simultaneously.
*   **Orbit:** Maintains a circular turn radius around a defined point by continuously controlling the radial error.

---

## 5. Secure Communications & Telemetry

The drone communicates using an SX1280 modem.
*   **Dual-Pipe:** Logically separates control/command packets (`0xAA`) and waypoint configuration packets (`0xAC`).
*   **Spread Spectrum (FHSS):** Automatic high-speed frequency hopping across 8 channels to evade interference.
*   **Cryptography:** All telemetry and commands use strong **XXTEA** cross-block encryption coupled with a CRC-16 checksum for authenticity and integrity.

---

## 6. Blackbox Logging

*   **Ping-Pong DMA Access:** Uses lock-free double buffers to write 32-byte logs to the W25Q32JVSS Flash memory. This ensures that data capture (attitude, vibration, motor states) does not halt the 1000Hz loop under any circumstances.
