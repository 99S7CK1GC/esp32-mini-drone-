# ESP32-C3 Micro Drone

A custom-built micro drone and dedicated remote controller, powered by the ESP32-C3 Super Mini microcontroller.

## Overview

This project contains the hardware design, circuit diagrams, and firmware for building a micro drone from scratch. The system uses the ESP-NOW protocol for low-latency, peer-to-peer communication between the custom remote controller and the drone.

## Project Structure

*   **`controller/`**: Contains the PlatformIO code, circuit diagrams, and images for the custom remote controller.
*   **`drone/`**: Contains the PlatformIO code, circuit diagrams, and images for the flight controller onboard the drone.
*   **`shared/`**: Contains shared code such as `protocol.h` which defines the ESP-NOW data structures used by both the drone and controller.
*   **`handout/`**: Contains the original project manual (`Micro_Drone_Manual.pdf`).

## Hardware Architecture

*   **MCU**: ESP32-C3 Super Mini (used in both drone and controller)
*   **Sensors**: MPU-6050 6-axis Gyroscope & Accelerometer
*   **Motors**: 8520 Coreless DC Motors
*   **Motor Drivers**: AO3400 N-channel MOSFETs with 1N4148 flyback diodes
*   **Communication**: ESP-NOW (2.4GHz)

> [!WARNING]
> **Critical Hardware Flaw:** Please read the `architecture_review.md` and `REMARKS.md` files before building. Using an AMS1117-3.3V LDO for the ESP32-C3 (as originally suggested in some manuals) may cause mid-flight brownouts due to voltage sag from the motors. It is highly recommended to use an ultra-low dropout (ULDO) regulator or a buck-boost converter instead.

## Firmware

The code for both the drone and the controller is written using the Arduino framework and managed via **PlatformIO**. 

*   The drone firmware implements a custom PID control loop (Rate Mode / Acro).
*   The code has been updated to use modern ESP32 Arduino Core v3.x APIs (e.g., modern `ledcAttach` and `ledcWrite`).

### How to Build

1.  Install [PlatformIO](https://platformio.org/).
2.  Open the `controller/code` or `drone/code` folder as a PlatformIO project.
3.  Build and upload to your ESP32-C3 board.

## Documentation

For a detailed review of the architecture, control loop, and tuning advice, see the included documentation:
*   [Architecture Review](architecture_review.md)
*   [Remarks](REMARKS.md)