#include "fusion_nav.h"
#include <math.h>

#if !defined(M_PI)
#define M_PI 3.14159265358979323846
#endif

static constexpr float kDegToRad = (float)(M_PI / 180.0);
static constexpr float kMaxDtS = 0.5f;
static constexpr float kGpsBlend = 0.35f;

/** WGS84 origin when GPS has no fix yet (West Lafayette, IN area). */
static constexpr double kDefaultOriginLat = 40.427837;
static constexpr double kDefaultOriginLon = -86.916360;

static double s_originLat = 0.0;
static double s_originLon = 0.0;
static bool s_originSet = false;
/** True if current origin is the default above; replaced once when GPS gets a valid fix. */
static bool s_originIsAssumed = false;

/** Use micros() so dt is non-zero when loop() runs faster than 1 kHz (millis() would give dt=0). */
static uint32_t s_lastUs = 0;

static float s_x = 0.f;
static float s_y = 0.f;
static float s_vx = 0.f;
static float s_vy = 0.f;

static float latLonToNorthM(double lat, double lat0) {
  return (float)((lat - lat0) * 111132.0);
}

static float latLonToEastM(double lon, double lon0, double lat0Deg) {
  const double cosLat = cos(lat0Deg * (M_PI / 180.0));
  return (float)((lon - lon0) * 111320.0 * cosLat);
}

/** Inverse of latLonToNorthM / latLonToEastM (small-angle local tangent plane, WGS84 ~). */
static void enuMetersToLatLon(float east_m, float north_m, double lat0_deg, double lon0_deg,
                              double *out_lat_deg, double *out_lon_deg) {
  constexpr double kMPerDegLat = 111132.0;
  const double cosLat = cos(lat0_deg * (M_PI / 180.0));
  const double kMPerDegLon = 111320.0 * cosLat;
  *out_lat_deg = lat0_deg + (double)north_m / kMPerDegLat;
  *out_lon_deg = lon0_deg + (kMPerDegLon > 1e-9 ? (double)east_m / kMPerDegLon : 0.0);
}

static void writeFusedGeo(FusedNav &out) {
  if (!s_originSet) {
    out.fused_coord_valid = false;
    out.fused_latitude = 0.0;
    out.fused_longitude = 0.0;
    return;
  }
  enuMetersToLatLon(s_x, s_y, s_originLat, s_originLon, &out.fused_latitude, &out.fused_longitude);
  out.fused_coord_valid = true;
}

static void integrateImu(const IMUData &imu, float dt) {
  const float psi = imu.heading * kDegToRad;
  const float c = cosf(psi);
  const float s = sinf(psi);
  const float ax = imu.linAccX;
  const float ay = imu.linAccY;
  const float a_east = ax * s + ay * c;
  const float a_north = ax * c - ay * s;

  s_vx += a_east * dt;
  s_vy += a_north * dt;
  s_x += s_vx * dt;
  s_y += s_vy * dt;
}

static float computeDtS(uint32_t now_us) {
  if (s_lastUs == 0) {
    s_lastUs = now_us;
    return 0.f;
  }
  uint32_t dus = now_us - s_lastUs;
  s_lastUs = now_us;
  float dt = dus * 1e-6f;
  if (dt < 0.f) {
    dt = 0.f;
  }
  if (dt > kMaxDtS) {
    dt = kMaxDtS;
  }
  return dt;
}

void fusionInit() {
  s_originLat = s_originLon = 0.0;
  s_originSet = false;
  s_originIsAssumed = false;
  s_lastUs = 0;
  s_x = s_y = s_vx = s_vy = 0.f;
}

void fusionUpdate(FusedNav &out, const GPSData &gps, const IMUData &imu, bool imuDataOk,
                  unsigned long now_ms) {
  (void)now_ms;

  const uint32_t nowUs = micros();
  const float dt = computeDtS(nowUs);

  out.dt_s = dt;
  out.nav_initialized = false;
  out.origin_set = s_originSet;
  out.gps_corrected_step = false;
  out.imu_predicted_step = false;
  out.x_m = out.y_m = 0.f;
  out.vx_m_s = out.vy_m_s = 0.f;
  out.fused_coord_valid = false;
  out.fused_latitude = 0.0;
  out.fused_longitude = 0.0;

  // First update: set ENU origin from GPS if available, else assumed default coordinates.
  if (!s_originSet) {
    if (gps.valid) {
      s_originLat = gps.latitude;
      s_originLon = gps.longitude;
      s_originIsAssumed = false;
    } else {
      s_originLat = kDefaultOriginLat;
      s_originLon = kDefaultOriginLon;
      s_originIsAssumed = true;
    }
    s_originSet = true;
    s_x = s_y = s_vx = s_vy = 0.f;
    s_lastUs = micros();
    out.dt_s = 0.f;
    out.origin_set = true;
    out.nav_initialized = true;
    out.x_m = s_x;
    out.y_m = s_y;
    out.vx_m_s = s_vx;
    out.vy_m_s = s_vy;
    out.gps_corrected_step = gps.valid;
    writeFusedGeo(out);
    return;
  }

  // Started with assumed origin; first valid GPS fix — snap origin and reset ENU drift.
  if (s_originIsAssumed && gps.valid) {
    s_originLat = gps.latitude;
    s_originLon = gps.longitude;
    s_originIsAssumed = false;
    s_x = s_y = s_vx = s_vy = 0.f;
    s_lastUs = micros();
  }

  // Origin fixed: GPS + IMU fusion
  out.origin_set = true;
  out.nav_initialized = true;

  float gx = 0.f, gy = 0.f;
  if (gps.valid) {
    gx = latLonToEastM(gps.longitude, s_originLon, s_originLat);
    gy = latLonToNorthM(gps.latitude, s_originLat);
  }

  if (imuDataOk && dt > 1e-5f) {
    integrateImu(imu, dt);
    out.imu_predicted_step = true;
  }

  if (gps.valid) {
    s_x = (1.f - kGpsBlend) * s_x + kGpsBlend * gx;
    s_y = (1.f - kGpsBlend) * s_y + kGpsBlend * gy;
    if (!imuDataOk) {
      s_vx = s_vy = 0.f;
    }
    out.gps_corrected_step = true;
  }

  out.x_m = s_x;
  out.y_m = s_y;
  out.vx_m_s = s_vx;
  out.vy_m_s = s_vy;
  writeFusedGeo(out);
}
