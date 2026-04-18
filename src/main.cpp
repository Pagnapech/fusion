#include <Arduino.h>
#include "imu_bno055.h"
#include "gps_neo6m.h"
#include "csv_logger.h"

IMUData imuData;
GPSData gpsData;

HardwareSerial GPSPort(1);

static const unsigned long kCsvIntervalMs = 2000;

void setup() {
  Serial.begin(115200);

  if (!imuInit(11, 10)) {
    Serial.println("ERROR: BNO055 not found!");
    while (1) {
      delay(1000);
    }
  }

  // GPS TX -> GPIO18
  // GPS RX -> not connected
  gpsInit(GPSPort, 18, -1, 9600);

  printCSVHeader(Serial);
}

void loop() {
  static unsigned long lastCsvMs = 0;

  gpsUpdate();

  unsigned long now = millis();
  if (now - lastCsvMs < kCsvIntervalMs) {
    return;
  }
  lastCsvMs = now;

  gpsRead(gpsData);
  if (!imuRead(imuData)) {
    imuData.heading = 0.0f;
  }

  printCSVRow(Serial, now, gpsData, imuData);
}
