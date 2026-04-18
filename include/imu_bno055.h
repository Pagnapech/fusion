#pragma once

#include <Arduino.h>

struct IMUData {
  float heading;
  float pitch;
  float roll;

  uint8_t calSys;
  uint8_t calGyro;
  uint8_t calAccel;
  uint8_t calMag;
};

bool imuInit(int sdaPin = 11, int sclPin = 10);
bool imuRead(IMUData &data);
void imuPrint(const IMUData &data);