#include "csv_logger.h"

void resetAccumulator(CsvAccumulator &acc) {
  acc.sampleCount = 0;
  acc.ekfLatDegSum = 0.0;
  acc.ekfLonDegSum = 0.0;
}

void addSampleToAccumulator(CsvAccumulator &acc, const EkfOutput &ekf) {
  acc.sampleCount++;
  acc.ekfLatDegSum += ekf.latitude_deg;
  acc.ekfLonDegSum += ekf.longitude_deg;
}

void printCSVHeader(Stream &out) {
  out.println(F("ekf_latitude_deg,ekf_longitude_deg"));
  // Full header (disabled):
  // out.println(F("ms,gps_valid,latitude,longitude,heading,speed_kmph,satellites,status,gps_x_m,gps_y_m,ekf_x_m,"
  //               "ekf_y_m,ekf_vx_mps,ekf_vy_mps,ekf_heading_deg"));
}

void printAveragedCsvRow(Stream &out, const CsvAccumulator &acc) {
  if (acc.sampleCount == 0) {
    out.print(0.0, 7);
    out.print(',');
    out.println(0.0, 7);
    return;
  }

  const double n = static_cast<double>(acc.sampleCount);
  out.print(acc.ekfLatDegSum / n, 7);
  out.print(',');
  out.println(acc.ekfLonDegSum / n, 7);
}
