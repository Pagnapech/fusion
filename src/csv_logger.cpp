#include "csv_logger.h"

void printCSVHeader(Stream &out) {
  out.println(F("fused_latitude_deg,fused_longitude_deg"));
}

void printCSVRow(Stream &out, unsigned long ms, const GPSData &gps, const IMUData &imu,
                 const FusedNav &nav, bool imu_hw_ok, bool imu_read_ok) {
  (void)ms;
  (void)gps;
  (void)imu;
  (void)imu_hw_ok;
  (void)imu_read_ok;

  // --- Optional CSV fields (commented; enable out.print lines to log again) ---
  // // Time of this log line (milliseconds since boot, millis()).
  // out.print(ms);
  // out.print(',');
  // // GPS reports a valid latitude/longitude fix (1) or not (0).
  // out.print(gps.valid ? 1 : 0);
  // out.print(',');
  // // Latitude in degrees (WGS84); 0 if no fix.
  // out.print(gps.latitude, 6);
  // out.print(',');
  // // Longitude in degrees (WGS84); 0 if no fix.
  // out.print(gps.longitude, 6);
  // out.print(',');
  // // BNO055 fusion heading (degrees, typically 0–360).
  // out.print(imu.heading, 2);
  // out.print(',');
  // // GPS ground speed (km/h); may be 0 if invalid.
  // out.print(gps.speedKmph, 2);
  // out.print(',');
  // // Number of satellites used (GPS message); 0 if not valid.
  // out.print(gps.satellites);
  // out.print(',');
  // // GPS pipeline state: 0=no serial, 1=receiving no fix, 2=valid fix (see GPSStatus).
  // out.print(static_cast<int>(gps.status));
  // out.print(',');
  // // IMU chip responded at boot during imuInit (1) or missing (0).
  // out.print(imu_hw_ok ? 1 : 0);
  // out.print(',');
  // // Last imuRead() in this loop succeeded (1) or failed/disabled (0).
  // out.print(imu_read_ok ? 1 : 0);
  // out.print(',');
  // // Linear acceleration X, gravity removed, sensor frame (m/s^2).
  // out.print(imu.linAccX, 3);
  // out.print(',');
  // // Linear acceleration Y (m/s^2).
  // out.print(imu.linAccY, 3);
  // out.print(',');
  // // Fused velocity east (m/s).
  // out.print(nav.vx_m_s, 3);
  // out.print(',');
  // // Fused velocity north (m/s).
  // out.print(nav.vy_m_s, 3);
  // out.print(',');
  // // Fusion has run with origin set and state valid (1).
  // out.print(nav.nav_initialized ? 1 : 0);
  // out.print(',');
  // // Local ENU origin was fixed from first valid GPS fix (1).
  // out.print(nav.origin_set ? 1 : 0);
  // out.print(',');
  // // Fusion time step used last update (seconds); capped in fusion_nav.
  // out.print(nav.dt_s, 4);
  // out.print(',');
  // // This fusion step applied GPS blend toward measured position (1).
  // out.print(nav.gps_corrected_step ? 1 : 0);
  // out.print(',');
  // // This fusion step integrated IMU linear accel for prediction (1).
  // out.println(nav.imu_predicted_step ? 1 : 0);

  // WGS84 fused position (ENU + origin; default origin if GPS had no fix at boot).
  out.print(nav.fused_latitude, 7);
  out.print(',');
  out.println(nav.fused_longitude, 7);
  // // Fused position east/north of local origin (meters, ENU) — same state as lat/lon above.
  // out.print(nav.x_m, 3);
  // out.print(',');
  // out.println(nav.y_m, 3);
}
