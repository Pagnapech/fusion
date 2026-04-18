#pragma once

#include <Arduino.h>
#include "gps_neo6m.h"
#include "imu_bno055.h"
#include "fusion_nav.h"

void printCSVHeader(Stream &out);
void printCSVRow(Stream &out, unsigned long ms, const GPSData &gps, const IMUData &imu,
                 const FusedNav &nav, bool imu_hw_ok, bool imu_read_ok);
