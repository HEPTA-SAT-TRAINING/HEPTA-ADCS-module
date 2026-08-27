#ifndef ANGULAR_ESTIMATION_H
#define ANGULAR_ESTIMATION_H

#include <Arduino.h>

#include "../../drv/imu9axis_bno055.h"

// Estimates relative Z-axis attitude by fusing gyro integration and magnetic
// heading. The magnetic heading removes long-term gyro drift while the gyro
// preserves fast motion.
class AngularEstimation {
 public:
  enum EstimationMode {
    ESTIMATION_GYRO_ONLY,
    ESTIMATION_MAG_ONLY,
    ESTIMATION_FUSION
  };

  static constexpr float GYRO_FILTER_ALPHA = 0.2f;
  static constexpr unsigned long MAX_GYRO_INTEGRATION_INTERVAL_MS = 100;
  static constexpr unsigned long MAG_SAMPLE_INTERVAL_MS = 50;
  static constexpr float MAG_CORRECTION_GAIN_PER_SEC = 0.5f;
  static constexpr float MIN_VALID_MAG_FIELD_UT = 10.0f;
  static constexpr float MAX_VALID_MAG_FIELD_UT = 100.0f;
  static constexpr float MAX_MAG_INNOVATION_DEG = 45.0f;
  // A positive body Z rotation makes the measured field rotate in the
  // opposite direction in body coordinates.
  static constexpr float MAG_HEADING_SIGN = -1.0f;
  void reset(unsigned long now_ms);
  void update(Bno055 &sensor, unsigned long now_ms);
  void set_mode(EstimationMode mode, unsigned long now_ms);
  EstimationMode mode() const;
  const char *mode_name() const;
  float yaw_deg() const;
  float yaw_rate_deg_per_sec() const;
  float magnetic_yaw_deg() const;
  bool magnetometer_is_valid() const;
  float error_deg(float target_yaw_deg) const;

 private:
  static float normalize_angle_deg(float angle_deg);

  float yaw_deg_ = 0.0f;
  float previous_gyro_z_deg_per_sec_ = 0.0f;
  float magnetic_reference_heading_deg_ = 0.0f;
  float magnetic_yaw_deg_ = 0.0f;
  unsigned long previous_sample_ms_ = 0;
  unsigned long previous_mag_sample_ms_ = 0;
  bool has_previous_sample_ = false;
  bool has_magnetic_reference_ = false;
  bool magnetometer_is_valid_ = false;
  EstimationMode mode_ = ESTIMATION_FUSION;
};

#endif  // ANGULAR_ESTIMATION_H
