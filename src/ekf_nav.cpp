#include "ekf_nav.h"
#include <math.h>
#include <string.h>

#if !defined(M_PI)
#define M_PI 3.14159265358979323846
#endif

static constexpr float kDegToRad = (float)(M_PI / 180.0);
static constexpr double kDefaultOriginLat = 40.427837;
static constexpr double kDefaultOriginLon = -86.916360;

static constexpr float kLowSpeedThreshMps = 0.2f;

// Process noise (tune)
static constexpr float kQPos = 0.02f;
static constexpr float kQVel = 0.15f;
// Measurement noise variance (m^2) ~ (3–5 m std for consumer GPS)
static constexpr float kRPos = 16.0f;

static double s_originLat = 0.0;
static double s_originLon = 0.0;
static bool s_originSet = false;
static bool s_originIsAssumed = false;

static float s_x[4] = {0.f, 0.f, 0.f, 0.f}; // e, n, ve, vn
static float s_P[16];                       // row-major 4x4

static void mat44_mul(const float *A, const float *B, float *C) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      float s = 0.f;
      for (int k = 0; k < 4; ++k) {
        s += A[i * 4 + k] * B[k * 4 + j];
      }
      C[i * 4 + j] = s;
    }
  }
}

static void mat44_transpose(const float *A, float *At) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      At[j * 4 + i] = A[i * 4 + j];
    }
  }
}

static void mat44_symmetrize(float *P) {
  for (int i = 0; i < 4; ++i) {
    for (int j = i + 1; j < 4; ++j) {
      float v = 0.5f * (P[i * 4 + j] + P[j * 4 + i]);
      P[i * 4 + j] = P[j * 4 + i] = v;
    }
  }
}

static void predict_covariance(float dt) {
  float F[16] = {
      1.f, 0.f, dt,  0.f,
      0.f, 1.f, 0.f, dt,
      0.f, 0.f, 1.f, 0.f,
      0.f, 0.f, 0.f, 1.f,
  };
  float FP[16];
  mat44_mul(F, s_P, FP);
  float Ft[16];
  mat44_transpose(F, Ft);
  float FPFt[16];
  mat44_mul(FP, Ft, FPFt);
  float Q[16] = {
      kQPos, 0.f,   0.f,   0.f,
      0.f,   kQPos, 0.f,   0.f,
      0.f,   0.f,   kQVel, 0.f,
      0.f,   0.f,   0.f,   kQVel,
  };
  for (int i = 0; i < 16; ++i) {
    s_P[i] = FPFt[i] + Q[i];
  }
  mat44_symmetrize(s_P);
}

static void gps_update(float ze, float zn) {
  const float y0 = ze - s_x[0];
  const float y1 = zn - s_x[1];
  const float p00 = s_P[0], p01 = s_P[1];
  const float p10 = s_P[4], p11 = s_P[5];
  const float p20 = s_P[8], p21 = s_P[9];
  const float p30 = s_P[12], p31 = s_P[13];

  const float s00 = p00 + kRPos;
  const float s01 = p01;
  const float s10 = p10;
  const float s11 = p11 + kRPos;
  const float det = s00 * s11 - s01 * s10;
  if (fabsf(det) < 1e-9f) {
    return;
  }
  const float inv_det = 1.f / det;
  const float i00 = s11 * inv_det;
  const float i01 = -s01 * inv_det;
  const float i10 = -s10 * inv_det;
  const float i11 = s00 * inv_det;

  const float k00 = p00 * i00 + p01 * i10;
  const float k01 = p00 * i01 + p01 * i11;
  const float k10 = p10 * i00 + p11 * i10;
  const float k11 = p10 * i01 + p11 * i11;
  const float k20 = p20 * i00 + p21 * i10;
  const float k21 = p20 * i01 + p21 * i11;
  const float k30 = p30 * i00 + p31 * i10;
  const float k31 = p30 * i01 + p31 * i11;

  s_x[0] += k00 * y0 + k01 * y1;
  s_x[1] += k10 * y0 + k11 * y1;
  s_x[2] += k20 * y0 + k21 * y1;
  s_x[3] += k30 * y0 + k31 * y1;

  float KH[16];
  memset(KH, 0, sizeof(KH));
  KH[0] = k00;
  KH[1] = k01;
  KH[4] = k10;
  KH[5] = k11;
  KH[8] = k20;
  KH[9] = k21;
  KH[12] = k30;
  KH[13] = k31;

  float IKH[16];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      const float id = (i == j) ? 1.f : 0.f;
      IKH[i * 4 + j] = id - KH[i * 4 + j];
    }
  }
  float Pnew[16];
  mat44_mul(IKH, s_P, Pnew);
  memcpy(s_P, Pnew, sizeof(s_P));
  mat44_symmetrize(s_P);
}

static float lat_lon_to_north_m(double lat, double lat0) {
  return (float)((lat - lat0) * 111132.0);
}

static float lat_lon_to_east_m(double lon, double lon0, double lat0_deg) {
  const double cosLat = cos(lat0_deg * (M_PI / 180.0));
  return (float)((lon - lon0) * 111320.0 * cosLat);
}

static void enu_to_lat_lon(float east_m, float north_m, double lat0_deg, double lon0_deg,
                           double *out_lat, double *out_lon) {
  constexpr double kMPerDegLat = 111132.0;
  const double cosLat = cos(lat0_deg * (M_PI / 180.0));
  const double kMPerDegLon = 111320.0 * cosLat;
  *out_lat = lat0_deg + (double)north_m / kMPerDegLat;
  *out_lon = lon0_deg + (kMPerDegLon > 1e-9 ? (double)east_m / kMPerDegLon : 0.0);
}

static float s_fallback_heading_deg = 0.f;

static float heading_from_velocity(float ve, float vn) {
  const float spd = hypotf(ve, vn);
  if (spd < kLowSpeedThreshMps) {
    return NAN;
  }
  float rad = atan2f(ve, vn);
  float deg = rad * (180.f / (float)M_PI);
  if (deg < 0.f) {
    deg += 360.f;
  }
  return deg;
}

static void init_P_large() {
  memset(s_P, 0, sizeof(s_P));
  s_P[0] = s_P[5] = 100.f;
  s_P[10] = s_P[15] = 25.f;
}

void ekfInit() {
  s_originLat = s_originLon = 0.0;
  s_originSet = false;
  s_originIsAssumed = false;
  memset(s_x, 0, sizeof(s_x));
  init_P_large();
}

void ekfGpsEnuMeters(const GPSData &gps, float *out_east_m, float *out_north_m) {
  if (!s_originSet || !gps.valid) {
    *out_east_m = 0.f;
    *out_north_m = 0.f;
    return;
  }
  *out_east_m = lat_lon_to_east_m(gps.longitude, s_originLon, s_originLat);
  *out_north_m = lat_lon_to_north_m(gps.latitude, s_originLat);
}

void ekfGetOutput(EkfOutput &out) {
  out.east_m = s_x[0];
  out.north_m = s_x[1];
  out.ve_mps = s_x[2];
  out.vn_mps = s_x[3];
  out.origin_set = s_originSet;
  if (!s_originSet) {
    out.latitude_deg = 0.0;
    out.longitude_deg = 0.0;
    out.heading_deg = 0.f;
    return;
  }
  enu_to_lat_lon(s_x[0], s_x[1], s_originLat, s_originLon, &out.latitude_deg, &out.longitude_deg);
  const float hv = heading_from_velocity(s_x[2], s_x[3]);
  if (!isnan(hv)) {
    out.heading_deg = hv;
  } else {
    out.heading_deg = s_fallback_heading_deg;
  }
}

void ekfStep(const GPSData &gps, const IMUData &imu, bool imu_ok, float dt_sec) {
  if (dt_sec > 0.5f) {
    dt_sec = 0.5f;
  }

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
    memset(s_x, 0, sizeof(s_x));
    init_P_large();
    /* Fall through: run predict + GPS update in the same step so the filter runs every dt
       instead of skipping one interval (previous early return left state stuck with no update). */
  }

  if (s_originIsAssumed && gps.valid) {
    s_originLat = gps.latitude;
    s_originLon = gps.longitude;
    s_originIsAssumed = false;
    memset(s_x, 0, sizeof(s_x));
    init_P_large();
  }

  float ae = 0.f;
  float an = 0.f;
  if (imu_ok && dt_sec > 1e-5f) {
    s_fallback_heading_deg = imu.heading;
    const float psi = imu.heading * kDegToRad;
    const float c = cosf(psi);
    const float s = sinf(psi);
    ae = imu.linAccX * s + imu.linAccY * c;
    an = imu.linAccX * c - imu.linAccY * s;
  }

  if (dt_sec > 1e-5f) {
    s_x[0] += s_x[2] * dt_sec;
    s_x[1] += s_x[3] * dt_sec;
    s_x[2] += ae * dt_sec;
    s_x[3] += an * dt_sec;
    predict_covariance(dt_sec);
  }

  if (gps.valid) {
    float ze = lat_lon_to_east_m(gps.longitude, s_originLon, s_originLat);
    float zn = lat_lon_to_north_m(gps.latitude, s_originLat);
    gps_update(ze, zn);
  }
}
