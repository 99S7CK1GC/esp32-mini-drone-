# Micro Drone Build: Architecture & Design Review

I've thoroughly analyzed the `Micro_Drone_Manual.pdf` and reviewed the system architecture, hardware choices, and firmware logic. This isn't just a copy-paste job—I wanted to make sure what we are building is robust, safe, and flight-capable.

Here are my remarks on the project's feasibility, along with a critical hardware flaw I identified in the manual, and how we can fix it.

---

## 1. Hardware Architecture Review

### The Good
*   **MCU (ESP32-C3 Super Mini)**: Excellent choice. It's incredibly small, lightweight, and the 160MHz RISC-V core is more than capable of running a 250Hz (4ms) flight loop. It has plenty of headroom for PID calculations, I2C polling, and handling incoming ESP-NOW packets.
*   **Motor Driver Circuit**: The use of AO3400 N-channel MOSFETs is perfect for coreless motors. With an $R_{ds(on)}$ of 28mΩ and a 5.7A max drain current, they will run completely cool while driving 8520 coreless motors (which typically draw 1.0A - 1.5A at full throttle). The inclusion of the 1N4148 flyback diode is essential to protect the FETs from inductive spikes.
*   **Radio Protocol**: ESP-NOW is the ideal protocol for this. It provides low-latency, connectionless, peer-to-peer communication between the controller and the drone without needing a WiFi router. A 500ms failsafe timeout is also perfectly implemented in the code to prevent flyaways.

### 🚨 THE CRITICAL FLAW: Power System Design 🚨
The manual recommends powering the ESP32-C3 logic rail from the 1S LiPo via an **AMS1117-3.3** Low Dropout (LDO) regulator. 
**This will likely cause mid-flight brownouts.**

*   **The Problem:** The AMS1117 is an older regulator with a high dropout voltage (typically 1.1V to 1.3V under load). 
*   **The Math:** A 1S LiPo is 4.2V fully charged but sags to 3.2V–3.5V when four coreless motors pull 4+ Amps. 
    `3.4V (sagging battery) - 1.1V (AMS1117 dropout) = 2.3V`
*   **The Result:** The ESP32-C3 requires a minimum of 3.0V to operate stably, especially when transmitting WiFi/ESP-NOW packets. If the voltage drops to 2.3V, the ESP32 will reset, the drone will drop out of the sky, and you'll lose control.

**✅ How to Fix It:**
Do not use the AMS1117. Instead, you need one of the following:
1.  **A modern ultra-low dropout (ULDO) regulator**: Something like the `RT9013-33` or `ME6211`, which have dropouts in the 100mV–200mV range.
2.  **A Buck-Boost Converter**: A tiny synchronous buck-boost module that will provide a solid 3.3V even if the LiPo drops down to 2.8V.

---

## 2. Firmware & Software Review

### The Control Loop (Project 17)
The manual instructs us to build a **Rate Mode (Acro)** controller. 
*   **Feasibility:** The code uses a classic PID algorithm and a fixed 4000µs (250Hz) loop timer using `micros()`. This is robust and standard for custom flight controllers.
*   **Sensors:** The MPU-6050 is polled via I2C at 400kHz. The code correctly subtracts the calibration offsets derived at startup. 
*   **Note on the Complementary Filter:** Interestingly, the manual explains a complementary filter for angle estimation (Project 8), but the final Project 17 code (Rate Mode) ignores it entirely. This is actually correct! Rate mode only cares about angular *velocity* (from the gyro), not absolute angle (from the accelerometer).

### Code Modernization (ESP32 Arduino Core 3.x)
I noticed the manual uses the `ledcSetup` and `ledcAttachPin` APIs. These were deprecated and removed in the latest ESP32 Arduino Core v3.0.0. 
*   **Action Taken:** I have proactively rewritten the `motors_init()` and `set_motor()` functions in the `drone/code/src/main.cpp` file to use the modern `ledcAttach()` and `ledcWrite()` APIs so the code will successfully compile on modern toolchains.

---

## 3. Next Steps & Tuning Advice

I am currently running a test compilation of the drone firmware in the background to ensure the environment (`platformio.ini`) and code are completely sound. 

Once you build the hardware (remember to fix the LDO issue!), here is my advice for tuning the PID:
1.  **Start with Kp:** The default is `1.5`. Since this is a micro drone, it has very low inertia and responds extremely fast. If it oscillates rapidly (buzzes), lower `Kp`. If it feels mushy, increase it.
2.  **Add Kd:** Add a tiny bit of `Kd` (e.g., `0.05`) to dampen the snap-back when you let go of the sticks. 
3.  **Keep Ki Low:** Micro drones don't need much `Ki` in rate mode unless they consistently drift away from the set angle over several seconds.

Let me know if you want to proceed with adding **Stabilise Mode** (Angle Mode) to the firmware, or if you need help flashing the ESP32s!
