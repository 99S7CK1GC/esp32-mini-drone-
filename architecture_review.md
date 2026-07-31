# Architecture & Design Review

This document provides a technical analysis of the micro drone's system architecture, hardware choices, and firmware logic based on the original project manual. It outlines key design decisions, highlights a critical hardware flaw to avoid, and provides guidance on flight tuning.

---

## 1. Hardware Architecture Review

### Design Highlights
*   **MCU (ESP32-C3 Super Mini)**: An excellent choice for a micro drone. It is extremely small and lightweight, and the 160MHz RISC-V core is highly capable of running a 250Hz (4ms) flight loop with ample headroom for PID calculations, I2C polling, and handling incoming ESP-NOW packets.
*   **Motor Driver Circuit**: The use of AO3400 N-channel MOSFETs is ideal for coreless motors. With an $R_{ds(on)}$ of 28mΩ and a 5.7A max drain current, they run completely cool while driving 8520 coreless motors (which typically draw 1.0A - 1.5A at full throttle). The inclusion of 1N4148 flyback diodes is essential to protect the FETs from inductive spikes.
*   **Radio Protocol**: ESP-NOW provides low-latency, connectionless, peer-to-peer communication between the controller and the drone without relying on a WiFi router. A 500ms failsafe timeout is implemented in the codebase to prevent flyaways if the connection drops.

### Power System Design

> [!CAUTION]
> **CRITICAL FLAW: Do not use the AMS1117-3.3 regulator.**
> The original project manual recommends powering the ESP32-C3 logic rail from the 1S LiPo via an AMS1117-3.3 Low Dropout (LDO) regulator. **This will likely cause mid-flight brownouts.**

**The Problem:** 
The AMS1117 is an older regulator with a high dropout voltage (typically 1.1V to 1.3V under load). A 1S LiPo is 4.2V fully charged but sags to 3.2V–3.5V when four coreless motors pull 4+ Amps. 

`3.4V (sagging battery) - 1.1V (AMS1117 dropout) = 2.3V`

The ESP32-C3 requires a minimum of 3.0V to operate stably, especially when transmitting WiFi/ESP-NOW packets. If the voltage drops to 2.3V, the ESP32 will reset, the drone will lose power, and control will be lost.

**The Solution:**
Replace the AMS1117 with one of the following alternatives:
1.  **Ultra-Low Dropout (ULDO) Regulator**: Components like the `RT9013-33` or `ME6211` feature dropouts in the 100mV–200mV range, keeping the voltage stable as the battery drains.
2.  **Buck-Boost Converter**: A tiny synchronous buck-boost module will provide a solid 3.3V even if the LiPo sags down to 2.8V.

---

## 2. Firmware & Software Review

### The Control Loop
The firmware utilizes a **Rate Mode (Acro)** controller. 
*   **Algorithm & Loop Time:** The code implements a classic PID algorithm with a fixed 4000µs (250Hz) loop timer using `micros()`. This approach is robust and standard for custom flight controllers.
*   **Sensors:** The MPU-6050 is polled via I2C at 400kHz. The code correctly subtracts calibration offsets derived at startup. 
*   **Angle Estimation:** While the original manual outlines a complementary filter for angle estimation, the Rate Mode code correctly bypasses it. Rate mode strictly utilizes angular *velocity* (from the gyroscope) and does not rely on absolute angle (from the accelerometer).

### Code Modernization (ESP32 Arduino Core 3.x)
The original project utilized `ledcSetup` and `ledcAttachPin` APIs, which have been deprecated and removed in ESP32 Arduino Core v3.0.0. 
*   **Updates Made:** The `motors_init()` and `set_motor()` functions in `drone/code/src/main.cpp` have been updated to use the modern `ledcAttach()` and `ledcWrite()` APIs, ensuring the project compiles successfully on modern toolchains.

---

## 3. Tuning Advice

Once the hardware is built and the LDO issue is mitigated, follow these steps to tune the PID controller for optimal flight:

1.  **Start with Proportional (Kp):** The default is `1.5`. Since this is a micro drone, it has very low inertia and responds extremely fast. If the drone oscillates rapidly (buzzes/shakes), lower Kp. If it feels mushy or slow to respond, increase it.
2.  **Add Derivative (Kd):** Introduce a small amount of Kd (e.g., `0.05`) to dampen the snap-back when releasing the controller sticks. This stops overshoot.
3.  **Keep Integral (Ki) Low:** Micro drones require very little Ki in rate mode unless they consistently drift away from the set angle over several seconds. Keep it minimal to prevent integral windup.
