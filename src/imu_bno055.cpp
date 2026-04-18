#include "imu_bno055.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Default BNO055 I2C address
static Adafruit_BNO055 bno(55, 0x28);
static bool imuReady = false;

bool imuInit(int sdaPin, int sclPin) {
  Wire.begin(sdaPin, sclPin);

  if (!bno.begin()) {
    imuReady = false;
    return false;
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  imuReady = true;
  return true;
}

bool imuRead(IMUData &data) {
  if (!imuReady) {
    return false;
  }

  sensors_event_t orientationData;
  bno.getEvent(&orientationData);

  data.heading = orientationData.orientation.x;
  data.pitch   = orientationData.orientation.y;
  data.roll    = orientationData.orientation.z;

  bno.getCalibration(
      &data.calSys,
      &data.calGyro,
      &data.calAccel,
      &data.calMag
  );

  return true;
}

void imuPrint(const IMUData &data) {
  Serial.print("Heading: ");
  Serial.print(data.heading);
  Serial.print("  Pitch: ");
  Serial.print(data.pitch);
  Serial.print("  Roll: ");
  Serial.println(data.roll);

  Serial.print("Calibration -> SYS:");
  Serial.print(data.calSys);
  Serial.print(" GYRO:");
  Serial.print(data.calGyro);
  Serial.print(" ACCEL:");
  Serial.print(data.calAccel);
  Serial.print(" MAG:");
  Serial.println(data.calMag);

  Serial.println("----------------------");
}