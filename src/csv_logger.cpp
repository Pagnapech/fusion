#include "csv_logger.h"

void printCSVHeader(Stream& out) {
  out.println(F("ms,gps_valid,latitude,longitude,heading,speed_kmph,satellites,status"));
}

void printCSVRow(Stream& out, unsigned long ms, const GPSData& gps, const IMUData& imu) {
  out.print(ms);
  out.print(',');
  out.print(gps.valid ? 1 : 0);
  out.print(',');
  out.print(gps.latitude, 6);
  out.print(',');
  out.print(gps.longitude, 6);
  out.print(',');
  out.print(imu.heading, 1);
  out.print(',');
  out.print(gps.speedKmph, 1);
  out.print(',');
  out.print(gps.satellites);
  out.print(',');
  out.println(static_cast<int>(gps.status));
}
