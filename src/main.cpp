#include <Arduino.h>
#include "imu_bno055.h"
#include "gps_neo6m.h"
#include "csv_logger.h"
#include "ekf_nav.h"
#include "rpi_uart.h"

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

  rpiUartInit();
  Serial.println(F("Raspberry Pi UART: position lines on UART2 (see rpi_uart.cpp for pins)"));
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

    rpiUartSendPosition(ekfOut.latitude_deg, ekfOut.longitude_deg);

    addSampleToAccumulator(gCsvAcc, ekfOut);
  }

  if (now - gLastCsvMs >= kCsvAverageIntervalMs) {
    gLastCsvMs = now;
    printAveragedCsvRow(Serial, gCsvAcc);
    resetAccumulator(gCsvAcc);
  }
}
