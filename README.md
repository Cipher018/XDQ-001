# XDQ-001

## Introduction

### What is XDQ-001?

The **XDQ-001**, or **Experimental Drone Quadcopter Model 001**, is a flight platform with original design. Its experimental objective is to achieve a FPV quadcopter capable of both acrobatic flight and cinematic filming. Furthermore, the XDQ-001 platform seeks to implement different filters from different sources for PID control and autonomous navigation by means of external or internal software.

### Whats New?

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

### Objectives

The primary objective is to develop a FPV quadcopter capable of both acrobatic flight and cinematic filming.

The secondary objectives are:

- Obtain an autocalibration system through PID and motor tests for 4 motors.
- Implementation of new control algorithms such as Mahony filters and 3-dimensional Kalman filters
- Implement autonomous functions such as waypoint navigation and orbiting
- Improvement of PID control
- Recording logs and telemetry data to flash memory

