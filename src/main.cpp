#include <Arduino.h>
#include "imu_bno055.h"
#include "gps_neo6m.h"
#include "csv_logger.h"
#include "ekf_nav.h"
#include "nav_state.h"
#include "mission_espnow.h"
#include "waypoint_nav.h"

IMUData imuData;
GPSData gpsData;

HardwareSerial GPSPort(1);

static const unsigned long kSampleIntervalMs = 100;
static const unsigned long kCsvAverageIntervalMs = 1000;

static bool gImuHwOk = false;

static CsvAccumulator gCsvAcc;
static unsigned long gLastSampleMs = 0;
static unsigned long gLastCsvMs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  gImuHwOk = imuInit(11, 10);
  if (!gImuHwOk) {
    Serial.println(F("WARN: BNO055 not found — IMU reads disabled; EKF uses GPS updates when a fix is available."));
  }

  gpsInit(GPSPort, 18, -1, 9600);
  ekfInit();
  resetAccumulator(gCsvAcc);

  gLastSampleMs = millis();
  gLastCsvMs = millis();

  printCSVHeader(Serial);

  missionEspNowInit();
  Serial.println(F("ESP-NOW mission link ready (see espnow_config.h for peer MAC + channel)"));
}

void loop() {
  gpsUpdate();

  const unsigned long now = millis();

  if (now - gLastSampleMs >= kSampleIntervalMs) {
    const float dt_sec = (now - gLastSampleMs) * 0.001f;
    gLastSampleMs = now;

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

    ekfStep(gpsData, imuData, imuReadOk, dt_sec);

    EkfOutput ekfOut{};
    ekfGetOutput(ekfOut);

    g_ekf_lat = ekfOut.latitude_deg;
    g_ekf_lon = ekfOut.longitude_deg;
    g_ekf_heading_deg = ekfOut.heading_deg;
    g_ekf_heading_valid = ekfOut.origin_set;

    addSampleToAccumulator(gCsvAcc, ekfOut);
  }

  runNavigationTick();

  if (now - gLastCsvMs >= kCsvAverageIntervalMs) {
    gLastCsvMs = now;
    printAveragedCsvRow(Serial, gCsvAcc);
    resetAccumulator(gCsvAcc);
  }
}
