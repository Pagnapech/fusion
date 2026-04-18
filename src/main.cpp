#include <Arduino.h>
#include "imu_bno055.h"
#include "gps_neo6m.h"
#include "csv_logger.h"
#include "fusion_nav.h"

IMUData imuData;
GPSData gpsData;
FusedNav fusedNav;

HardwareSerial GPSPort(1);

static const unsigned long kCsvIntervalMs = 2000;

static bool gImuHwOk = false;

void setup() {
  Serial.begin(115200);
  delay(200);

  gImuHwOk = imuInit(11, 10);
  if (!gImuHwOk) {
    Serial.println(F("WARN: BNO055 not found — IMU reads disabled; fusion uses GPS only when a fix is available."));
  }

  gpsInit(GPSPort, 18, -1, 9600);
  fusionInit();

  printCSVHeader(Serial);
}

void loop() {
  static unsigned long lastCsvMs = 0;

  gpsUpdate();

  unsigned long now = millis();

  gpsRead(gpsData);

  bool imuReadOk = false;
  if (gImuHwOk) {
    imuReadOk = imuRead(imuData);
  }
  if (!imuReadOk) {
    imuData.heading = 0.0f;
    imuData.pitch = 0.0f;
    imuData.roll = 0.0f;
    imuData.linAccX = 0.0f;
    imuData.linAccY = 0.0f;
    imuData.linAccZ = 0.0f;
  }

  fusionUpdate(fusedNav, gpsData, imuData, imuReadOk, now);

  if (now - lastCsvMs < kCsvIntervalMs) {
    return;
  }
  lastCsvMs = now;

  printCSVRow(Serial, now, gpsData, imuData, fusedNav, gImuHwOk, imuReadOk);
}
