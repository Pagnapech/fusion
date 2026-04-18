```markdown
# AI Agent Context: ESP32-S3 Baseline Implementation Plan

## Project Overview
Create a baseline hardware testing firmware for an ESP32-S3 interfacing with a NEO-6M GPS and a BNO055 9-Axis IMU. The goal is to establish stable, non-blocking communication with both sensors and selectively display their data on the Serial Monitor without race conditions or interleaved garbled text.

## Hardware & Pin Constraints
* **MCU:** ESP32-S3 DevKitC-1
* **IMU:** BNO055 via I2C (SCL: GPIO 9, SDA: GPIO 10).
* **GPS:** NEO-6M via Hardware UART (ESP RX: GPIO 17, ESP TX: GPIO 18). Baud rate: 9600.
* **Logic Level:** All data lines are 3.3V. 
* **Libraries:** `Adafruit_BNO055`, `TinyGPSPlus`.

## Software Architecture Rules (Strict)
1. **No Blocking Code:** Do not use `delay()`. All timing must be handled using `millis()`.
2. **Background Polling:** The `loop()` must constantly read from the GPS hardware serial to feed the `TinyGPS++` object. It must also poll the IMU at a reasonable interval.
3. **State-Driven Output:** To prevent the Serial Monitor from being flooded or interleaving data, implement a state machine for printing:
   * State 0: Idle (Print nothing, just poll sensors).
   * State 1: Print IMU Data only (e.g., Euler angles).
   * State 2: Print GPS Data only (e.g., Lat, Lon, Satellites).
4. **Interactive Control:** Read `Serial` input to allow the user to type '0', '1', or '2' to switch the active display state in real-time.
5. **Print Throttling:** Ensure the active print state only outputs data to the Serial Monitor every ~500ms to keep it readable.
6. **Simplicity:** Keep the code minimal, confined to `main.cpp` for this baseline test. Do not over-engineer with RTOS tasks unless absolutely necessary.

## Implementation Phases
When requested to generate code, follow these phases one by one:
* **Phase 1: I2C Scanner & IMU Init:** Initialize `Wire` on pins 9/10, initialize BNO055, and print Euler angles to test.
* **Phase 2: UART & GPS Init:** Initialize `Serial1` on 17/18, feed data to `TinyGPSPlus`, and print parsed coordinates.
* **Phase 3: Integration:** Combine Phase 1 and 2 using the State-Driven Output architecture defined above.