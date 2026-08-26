#include "angular_estimation.h"

extern float gyro_bias_z_deg_per_sec;
extern bool gyro_bias_correction_enabled;
extern bool magnetic_calibration_enabled;
extern float magnetic_offset_x_ut;
extern float magnetic_offset_y_ut;
extern float magnetic_scale_x;
extern float magnetic_scale_y;

void AngularEstimation::reset(unsigned long now_ms) {
  yaw_deg_ = 0.0f;
  previous_gyro_z_deg_per_sec_ = 0.0f;
  magnetic_reference_heading_deg_ = 0.0f;
  magnetic_yaw_deg_ = 0.0f;
  previous_sample_ms_ = now_ms;
  previous_mag_sample_ms_ = now_ms - MAG_SAMPLE_INTERVAL_MS;
  has_previous_sample_ = false;
  has_magnetic_reference_ = false;
  magnetometer_is_valid_ = false;
}

void AngularEstimation::update(Bno055 &sensor, unsigned long now_ms,
                               bool apply_magnetic_correction) {
  float gyro_x_deg_per_sec;
  float gyro_y_deg_per_sec;
  float gyro_z_deg_per_sec;
  if (!sensor.sen_gyro(&gyro_x_deg_per_sec, &gyro_y_deg_per_sec,
                       &gyro_z_deg_per_sec)) {
    return;
  }
  if (gyro_bias_correction_enabled) {
    gyro_z_deg_per_sec -= gyro_bias_z_deg_per_sec;
  }

  float dt_sec = 0.0f;
  if (has_previous_sample_) {
    //荳谺｡繝ｭ繝ｼ繝代せ繝輔ぅ繝ｫ繧ｿ縺ｧ繧ｸ繝｣繧､繝ｭ縺ｮ繝弱う繧ｺ繧剃ｽ取ｸ帙☆繧九ゅヵ繧｣繝ｫ繧ｿ菫よ焚縺ｯGYRO_FILTER_ALPHA縺ｧ隱ｿ謨ｴ縺吶ｋ縲・
    gyro_z_deg_per_sec = GYRO_FILTER_ALPHA * gyro_z_deg_per_sec + (1.0f - GYRO_FILTER_ALPHA) * previous_gyro_z_deg_per_sec_;
    const unsigned long sample_interval_ms = now_ms - previous_sample_ms_;
    if (sample_interval_ms <= MAX_GYRO_INTEGRATION_INTERVAL_MS) {
      dt_sec = sample_interval_ms / 1000.0f;
      //蜿ｰ蠖｢遨榊・
      yaw_deg_ += (previous_gyro_z_deg_per_sec_ + gyro_z_deg_per_sec) * 0.5f * dt_sec;
      yaw_deg_ = normalize_angle_deg(yaw_deg_);
    }
  }

  previous_gyro_z_deg_per_sec_ = gyro_z_deg_per_sec;
  previous_sample_ms_ = now_ms;
  has_previous_sample_ = true;

  if (now_ms - previous_mag_sample_ms_ < MAG_SAMPLE_INTERVAL_MS) return;

  const unsigned long mag_interval_ms = now_ms - previous_mag_sample_ms_;
  previous_mag_sample_ms_ = now_ms;

  //逎∵ｰ励そ繝ｳ繧ｵ縺ｮ蛟､繧貞叙蠕励＠・悟慍逎∵ｰ・蝓ｺ貅・縺ｨ繝ｪ繧｢繝ｫ繧ｿ繧､繝縺ｮ蝨ｰ逎∵ｰ励・蜷代″縺九ｉ・後Κ繝ｼ隗偵ｒ邂怜・
  //繧ｸ繝｣繧､繝ｭ繧ｻ繝ｳ繧ｵ縺ｮ蛟､縺ｫ・悟慍逎∵ｰ励Κ繝ｼ隗偵・隱､蟾ｮ繧定｣懈ｭ｣
  float mag_x_ut;
  float mag_y_ut;
  float mag_z_ut;

  if (!sensor.sen_mag(&mag_x_ut, &mag_y_ut, &mag_z_ut)) {
    magnetometer_is_valid_ = false;
    return;
  }
  if (magnetic_calibration_enabled) {
    mag_x_ut = (mag_x_ut - magnetic_offset_x_ut) * magnetic_scale_x;
    mag_y_ut = (mag_y_ut - magnetic_offset_y_ut) * magnetic_scale_y;
  }

  const float field_strength_ut = sqrtf(mag_x_ut * mag_x_ut + mag_y_ut * mag_y_ut + mag_z_ut * mag_z_ut);
  const bool field_is_valid =
      isfinite(field_strength_ut) &&
      field_strength_ut >= MIN_VALID_MAG_FIELD_UT &&
      field_strength_ut <= MAX_VALID_MAG_FIELD_UT;

  if (!field_is_valid) {
    magnetometer_is_valid_ = false;
    return;
  }

  const float magnetic_heading_deg = normalize_angle_deg(MAG_HEADING_SIGN * atan2f(mag_y_ut, mag_x_ut) * 180.0f / PI);
  if (!has_magnetic_reference_) {
    magnetic_reference_heading_deg_ = magnetic_heading_deg;
    magnetic_yaw_deg_ = 0.0f;
    has_magnetic_reference_ = true;
    magnetometer_is_valid_ = true;
    return;
  }

  magnetic_yaw_deg_ = normalize_angle_deg(magnetic_heading_deg - magnetic_reference_heading_deg_);
  const float innovation_deg = normalize_angle_deg(magnetic_yaw_deg_ - yaw_deg_);

  if (fabsf(innovation_deg) > MAX_MAG_INNOVATION_DEG) {
    magnetometer_is_valid_ = false;
    return;
  }

  // The reaction-wheel motor produces a strong, changing local magnetic
  // field. Its heading can pass the simple strength/innovation checks while
  // still being wrong, which drags the estimated yaw back toward zero and
  // makes the controller keep rotating past its target. Keep sampling for
  // diagnostics, but use gyro-only yaw while the wheel is under control.
  if (!apply_magnetic_correction) {
    magnetometer_is_valid_ = false;
    return;
  }

  const float correction_gain = constrain(MAG_CORRECTION_GAIN_PER_SEC * mag_interval_ms / 1000.0f, 0.0f, 0.1f);
  yaw_deg_ = normalize_angle_deg(yaw_deg_ + correction_gain * innovation_deg);
  magnetometer_is_valid_ = true;
}

float AngularEstimation::yaw_deg() const { return yaw_deg_; }
float AngularEstimation::yaw_rate_deg_per_sec() const {
  return previous_gyro_z_deg_per_sec_;
}
float AngularEstimation::magnetic_yaw_deg() const {
  return magnetic_yaw_deg_;
}
bool AngularEstimation::magnetometer_is_valid() const {
  return magnetometer_is_valid_;
}

float AngularEstimation::error_deg(float target_yaw_deg) const {
  return normalize_angle_deg(target_yaw_deg - yaw_deg_);
}

float AngularEstimation::normalize_angle_deg(float angle_deg) {
  while (angle_deg >= 180.0f) angle_deg -= 360.0f;
  while (angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

