#pragma once

#include <Arduino.h>
#include "gps_neo6m.h"
#include "imu_bno055.h"

/** 2D EKF state in local ENU (m, m/s). Origin matches fusion V2 default / GPS snap policy. */
struct EkfOutput {
  float east_m;
  float north_m;
  float ve_mps;
  float vn_mps;
  /** Course from velocity (deg, 0–360); IMU heading if speed low. */
  float heading_deg;
  double latitude_deg;
  double longitude_deg;
  bool origin_set;
};

void ekfInit();
void ekfStep(const GPSData &gps, const IMUData &imu, bool imu_ok, float dt_sec);
void ekfGetOutput(EkfOutput &out);

/** GPS position in same ENU frame as EKF (meters). Requires ekf origin; else 0,0. */
void ekfGpsEnuMeters(const GPSData &gps, float *out_east_m, float *out_north_m);
