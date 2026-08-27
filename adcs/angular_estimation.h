#ifndef ANGULAR_ESTIMATION_H
#define ANGULAR_ESTIMATION_H

#include <Arduino.h>

class HeptaSensor;

class AngularEstimation {
 public:
  enum Mode { GYRO_ONLY, MAGNETOMETER_ONLY, FUSED };

  void reset(unsigned long now_ms);
  void set_mode(Mode mode, unsigned long now_ms);
  void update(HeptaSensor &sensor, unsigned long now_ms);
  Mode mode() const { return mode_; }
  const char *mode_name() const;
  float yaw_deg() const { return yaw_deg_; }
  float gyro_x() const { return gx_; }
  float gyro_y() const { return gy_; }
  float gyro_z() const { return gz_; }
  float mag_x() const { return mx_; }
  float mag_y() const { return my_; }
  float mag_z() const { return mz_; }

 private:
  static float normalize(float angle);
  void update_magnetic_yaw(bool magnetometer_ok);
  bool apply_magnetometer_only_mode();
  void integrate_gyro_yaw(bool gyro_ok, unsigned long dt_ms, float dt_sec);
  void fuse_magnetic_yaw(float dt_sec);
  Mode mode_ = FUSED;
  float yaw_deg_ = 0, magnetic_yaw_deg_ = 0, magnetic_reference_deg_ = 0;
  float gx_ = 0, gy_ = 0, gz_ = 0, previous_gz_ = 0;
  float mx_ = 0, my_ = 0, mz_ = 0;
  unsigned long previous_ms_ = 0;
  bool has_gyro_ = false, has_magnetic_reference_ = false;
};

#endif
