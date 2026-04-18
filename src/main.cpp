#include <Arduino.h>
#include "imu_bno055.h"
#include "gps_neo6m.h"

IMUData imuData;
GPSData gpsData;

HardwareSerial GPSPort(1);

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting modular BNO055 + Neo-6M test...");

  if (!imuInit(11, 10)) {
    Serial.println("ERROR: BNO055 not found!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("BNO055 initialized successfully.");

  // GPS TX -> GPIO18
  // GPS RX -> not connected
  gpsInit(GPSPort, 18, -1, 9600);

  Serial.println("Neo-6M initialized.");
}

void loop() {
  static unsigned long lastPrint = 0;

  gpsUpdate();

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    if (imuRead(imuData)) {
      imuPrint(imuData);
    } else {
      Serial.println("IMU read failed.");
    }

    gpsRead(gpsData);
    gpsPrint(gpsData);
  }
}