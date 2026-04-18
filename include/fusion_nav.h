#pragma once

#include <Arduino.h>
#include "gps_neo6m.h"
#include "imu_bno055.h"

/**
 * Local tangent-plane state (ENU): x = east (m), y = north (m), velocities in m/s.
 *
 * Origin is set on the first fusion update: if GPS has a valid fix, origin = GPS lat/lon;
 * otherwise origin defaults to West Lafayette, IN (see fusion_nav.cpp). When a fix appears
 * later after using the default, origin snaps to GPS once and ENU state resets.
 *
 * fused_latitude / fused_longitude: WGS84 degrees from ENU + origin (valid once origin_set).
 */
struct FusedNav {
  float x_m;
  float y_m;
  float vx_m_s;
  float vy_m_s;
  float dt_s;
  bool nav_initialized;
  bool origin_set;
  bool gps_corrected_step;
  bool imu_predicted_step;
  /** True when fused lat/lon are meaningful (GPS origin has been captured). */
  bool fused_coord_valid;
  double fused_latitude;
  double fused_longitude;
};

void fusionInit();
void fusionUpdate(FusedNav &out, const GPSData &gps, const IMUData &imu, bool imuDataOk,
                  unsigned long now_ms);
