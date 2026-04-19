#pragma once

/**
 * Fused navigation state updated by the EKF path in main (each ~100 ms tick).
 * Used by waypoint navigation and ESP-NOW status.
 */
extern double g_ekf_lat;
extern double g_ekf_lon;
extern float g_ekf_heading_deg;
/** When false, skip heading-based steering (still navigate using bearing for progress). */
extern bool g_ekf_heading_valid;
