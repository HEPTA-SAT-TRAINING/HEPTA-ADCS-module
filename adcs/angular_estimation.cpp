#include "angular_estimation.h"
#include "../../src/hepta_sat/hepta_sensor.h"

float AngularEstimation::normalize(float angle) {
  while (angle >= 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void AngularEstimation::reset(unsigned long now_ms) {
  yaw_deg_ = magnetic_yaw_deg_ = magnetic_reference_deg_ = 0;
  previous_gz_ = 0;
  previous_ms_ = now_ms;
  has_gyro_ = has_magnetic_reference_ = false;
}

void AngularEstimation::set_mode(Mode mode, unsigned long now_ms) {
  mode_ = mode;
  reset(now_ms);
}

const char *AngularEstimation::mode_name() const {
  if (mode_ == GYRO_ONLY) return "GYRO";
  if (mode_ == MAGNETOMETER_ONLY) return "MAG";
  return "FUSED";
}
