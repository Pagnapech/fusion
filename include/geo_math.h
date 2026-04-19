#pragma once

/** WGS84 great-circle distance in meters (haversine). */
double geo_haversine_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

/**
 * Initial bearing from (lat1,lon1) to (lat2,lon2), degrees clockwise from north, range [0, 360).
 */
double geo_bearing_deg(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

double geo_deg2rad(double deg);
double geo_rad2deg(double rad);

/** Wrap angle to [-180, 180] degrees. */
double geo_normalize_angle_180_deg(double deg);
