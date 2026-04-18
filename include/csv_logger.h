#pragma once

#include <Arduino.h>
#include "gps_neo6m.h"
#include "imu_bno055.h"

void printCSVHeader(Stream& out);
void printCSVRow(Stream& out, unsigned long ms, const GPSData& gps, const IMUData& imu);
