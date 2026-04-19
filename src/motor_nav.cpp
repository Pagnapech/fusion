#include "motor_nav.h"
#include <Arduino.h>
#include <math.h>

// --- Tune for your platform (placeholder) ---
static constexpr float kBaseSpeed = 0.35f;   // normalized -1..1 forward command
static constexpr float kTurnKp = 0.02f;      // steering gain on heading error (deg -> differential)
static constexpr float kMaxMotor = 1.0f;

void stopMotors() {
  // TODO: set left/right motor PWM or H-bridge to neutral/stop
  // Example: ledcWrite(LEFT_CH, 0); ledcWrite(RIGHT_CH, 0);
}

void navigateTowardWaypoint(double dist_m, double heading_error_deg) {
  (void)dist_m;  // could slow down when close; proportional example uses heading only

  const float turn = (float)(kTurnKp * heading_error_deg);
  float left = kBaseSpeed - turn;
  float right = kBaseSpeed + turn;

  if (left > kMaxMotor) {
    left = kMaxMotor;
  }
  if (left < -kMaxMotor) {
    left = -kMaxMotor;
  }
  if (right > kMaxMotor) {
    right = kMaxMotor;
  }
  if (right < -kMaxMotor) {
    right = -kMaxMotor;
  }

  // TODO: apply `left` / `right` to motor drivers (-1..1 normalized duty)
  (void)left;
  (void)right;
}
