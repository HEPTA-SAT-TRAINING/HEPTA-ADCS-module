#ifndef ADCS_CONTROL_H
#define ADCS_CONTROL_H

#include <Arduino.h>

#include "angular_estimation.h"
#include "../../drv/imu9axis_bno055.h"
#include "../../drv/unit_rolleri2c.hpp"
#include "../../drv/unit_roller_common.hpp"

class AdcsControl {
 public:
  static constexpr int32_t MAX_WHEEL_SPEED_RPM = 600;
  static constexpr float CONTROL_SWITCH_THRESHOLD = 2.0f;
  static constexpr float MEDIUM_STEP_THRESHOLD = 10.0f;
  static constexpr float LARGE_STEP_THRESHOLD = 30.0f;
  static constexpr int32_t SMALL_SPEED_STEP_RPM = 5;
  static constexpr int32_t MEDIUM_SPEED_STEP_RPM = 20;
  static constexpr int32_t LARGE_SPEED_STEP_RPM = 60;
  static constexpr int32_t UNLOAD_START_SPEED_RPM = 590;
  static constexpr int32_t UNLOAD_SPEED_STEP_RPM = 20;
  static constexpr int32_t UNLOAD_FINISHED_SPEED_RPM = 10;
  static constexpr float UNLOAD_MIN_ERROR_DEG = 5.0f;
  static constexpr float MOVING_AWAY_YAW_RATE_DEG_PER_SEC = 1.0f;
  static constexpr unsigned long SATURATION_CONFIRM_MS = 1000;
  static constexpr unsigned long SETTLING_CONFIRM_MS = 1000;
  static constexpr float SETTLED_ANGLE_ERROR_DEG = 1.0f;
  static constexpr float SETTLED_YAW_RATE_DEG_PER_SEC = 0.5f;
  static constexpr unsigned long TARGET_HOLD_CONFIRM_MS = 1000;
  static constexpr float TARGET_HOLD_EXIT_ERROR_DEG = 3.0f;
  static constexpr unsigned long TARGET_HOLD_EXIT_CONFIRM_MS = 500;
  static constexpr unsigned long SPEED_COMMAND_STEP_INTERVAL_MS = 250;
  static constexpr unsigned long WHEEL_COMMAND_WRITE_INTERVAL_MS = 1000;
  static constexpr int32_t WHEEL_COMMAND_MIN_CHANGE_RPM = 60;
  static constexpr int32_t WHEEL_COMMAND_MAX_STEP_RPM = 60;
  static constexpr unsigned long WHEEL_READBACK_INTERVAL_MS = 250;
  static constexpr int32_t MAX_VALID_WHEEL_READBACK_RPM = 700;
  static constexpr int32_t MAX_VALID_READBACK_JUMP_RPM = 300;
  static constexpr int32_t MAX_READBACK_COMMAND_ERROR_RPM = 200;
  static constexpr int32_t SPEED_MODE_MAX_CURRENT_REGISTER = 50000;
  static constexpr int32_t WHEEL_STOPPED_THRESHOLD_RPM = 10;
  static constexpr unsigned long WHEEL_RECOVERY_DELAY_MS = 2000;
  static constexpr int32_t MAX_CURRENT_REGISTER = 100000;
  static constexpr float MAX_KP_MA_PER_DEG = 1000.0f;
  static constexpr float MAX_KD_MA_PER_DEG_PER_SEC = 1000.0f;
  static constexpr float CURRENT_REGISTER_PER_MA = 100.0f;

  String start(AngularEstimation &estimation, UnitRollerI2C &wheel,
               bool &wheel_is_running, unsigned long now_ms);
  String stop(UnitRollerI2C &wheel, bool &wheel_is_running);
  void update(AngularEstimation &estimation, Bno055 &sensor,
              UnitRollerI2C &wheel, unsigned long now_ms);

  void set_target_angle_deg(float target_angle_deg);
  bool set_proportional_gain(float kp_ma_per_deg);
  bool set_derivative_gain(float kd_ma_per_deg_per_sec);

  bool is_enabled() const;
  float target_angle_deg() const;
  float proportional_gain_ma_per_deg() const;
  float derivative_gain_ma_per_deg_per_sec() const;
  float error_deg(float estimated_yaw_deg) const;
  float current_command_ma() const;
  float current_readback_ma() const;
  float friction_compensation_ma() const;
  bool current_readback_is_valid() const;
  int32_t wheel_speed_rpm() const;
  bool wheel_readback_is_valid() const;
  float wheel_speed_command_rpm() const;
  int32_t last_sent_wheel_speed_rpm() const;
  const char *control_state_name() const;

 private:
  enum ControlState {
    CONTROL_STATE_NORMAL,
    CONTROL_STATE_SATURATED,
    CONTROL_STATE_UNLOADING,
    CONTROL_STATE_SETTLING,
    CONTROL_STATE_TARGET_HOLD
  };

  static float normalize_angle_deg(float angle_deg);
  float calculate_body_pd_command(
      float estimated_yaw_deg, float yaw_rate_deg_per_sec) const;

  bool enabled_ = false;
  float target_angle_deg_ = 0.0f;
  float kp_ma_per_deg_ = 2.0f;
  float kd_ma_per_deg_per_sec_ = 3.0f;
  int32_t current_command_ = 0;
  int32_t current_readback_ = 0;
  int32_t wheel_speed_rpm_ = 0;
  float wheel_speed_command_rpm_ = 0.0f;
  unsigned long previous_speed_step_ms_ = 0;
  unsigned long previous_wheel_command_write_ms_ = 0;
  int32_t last_sent_wheel_speed_rpm_ = 0;
  unsigned long previous_wheel_readback_ms_ = 0;
  unsigned long saturation_started_ms_ = 0;
  unsigned long settling_started_ms_ = 0;
  unsigned long target_hold_candidate_started_ms_ = 0;
  unsigned long target_hold_exit_started_ms_ = 0;
  unsigned long wheel_tracking_error_started_ms_ = 0;
  bool wheel_is_recovering_ = false;
  ControlState control_state_ = CONTROL_STATE_NORMAL;
};

#endif  // ADCS_CONTROL_H
