# PlatformIO Environment Setup for ESP32-S3

This document outlines the environment configuration for Cursor + PlatformIO to ensure the ESP32-S3 DevKitC-1 communicates properly with the NEO-6M GPS and BNO055 IMU, specifically addressing the native USB serial requirements of the S3.

## 1. Project Initialization
1. Open Cursor and navigate to the PlatformIO Home screen.
2. Click **New Project**.
   - **Name:** `ESP32_Telemetry_Node`
   - **Board:** `Espressif ESP32-S3-DevKitC-1-N8` (Standard DevKitC-1 is fine)
   - **Framework:** `Arduino`
3. Choose your project location and click **Finish**.

## 2. Configuring `platformio.ini`
The ESP32-S3 uses a native USB hardware peripheral. If you do not set the USB build flags correctly, `Serial.print()` will not output to the Cursor serial monitor. Replace your `platformio.ini` contents with this:

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

; Serial Monitor Options
monitor_speed = 115200
monitor_rts = 0
monitor_dtr = 0

; Crucial Build Flags for ESP32-S3 Native USB
build_flags = 
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1 

; Library Dependencies
lib_deps =
    ; GPS Parsing
    mikalhart/TinyGPSPlus 
    
    ; IMU Sensor
    adafruit/Adafruit BNO055 
    adafruit/Adafruit Unified Sensor 