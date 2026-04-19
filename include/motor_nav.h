#pragma once

/**
 * Placeholder motor / steering layer for differential drive.
 * Replace PWM / GPIO writes with your hardware drivers.
 */
void stopMotors();

/**
 * @param dist_m distance to active waypoint (meters)
 * @param heading_error_deg bearing minus heading, normalized to [-180, 180]; use 0 if no heading
 */
void navigateTowardWaypoint(double dist_m, double heading_error_deg);
