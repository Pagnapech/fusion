#include "geo_math.h"
#include <math.h>

#if !defined(M_PI)
#define M_PI 3.14159265358979323846
#endif

static const double kEarthRadiusM = 6371000.0;

double geo_deg2rad(double deg) { return deg * (M_PI / 180.0); }

double geo_rad2deg(double rad) { return rad * (180.0 / M_PI); }

double geo_normalize_angle_180_deg(double deg) {
  while (deg > 180.0) {
    deg -= 360.0;
  }
  while (deg < -180.0) {
    deg += 360.0;
  }
  return deg;
}

double geo_haversine_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
  const double phi1 = geo_deg2rad(lat1_deg);
  const double phi2 = geo_deg2rad(lat2_deg);
  const double dphi = geo_deg2rad(lat2_deg - lat1_deg);
  const double dlambda = geo_deg2rad(lon2_deg - lon1_deg);

  const double a = sin(dphi * 0.5) * sin(dphi * 0.5) + cos(phi1) * cos(phi2) * sin(dlambda * 0.5) * sin(dlambda * 0.5);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return kEarthRadiusM * c;
}

double geo_bearing_deg(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
  const double phi1 = geo_deg2rad(lat1_deg);
  const double phi2 = geo_deg2rad(lat2_deg);
  const double dlambda = geo_deg2rad(lon2_deg - lon1_deg);

  const double y = sin(dlambda) * cos(phi2);
  const double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dlambda);
  double brg = atan2(y, x);
  brg = geo_rad2deg(brg);
  brg = fmod(brg + 360.0, 360.0);
  return brg;
}
