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
  static constexpr float PD_TO_SPEED_STEP_RPM = 0.5f;
  static constexpr float MIN_SPEED_STEP_RPM = 1.0f;
  static constexpr float MAX_SPEED_STEP_RPM = 20.0f;
  static constexpr float MAX_MANEUVER_YAW_RATE_DEG_PER_SEC = 8.0f;
  static constexpr float BRAKING_ACCEL_DEG_PER_SEC2 = 2.0f;
  static constexpr float BRAKING_MARGIN_DEG = 3.0f;
  static constexpr int32_t BRAKING_RESERVE_RPM = 120;
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
  static constexpr int32_t WHEEL_COMMAND_MIN_CHANGE_RPM = 1;
  static constexpr int32_t WHEEL_COMMAND_MAX_STEP_RPM = 20;
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
  static constexpr float MAX_CONTROL_CURRENT_MA = 250.0f;
  static constexpr float MAX_CURRENT_SLEW_MA_PER_UPDATE = 25.0f;
  // A small breakaway current is used only close to the target.  The former
  // 120 mA step made a small hand disturbance produce a large limit cycle.
  static constexpr float FRICTION_COMPENSATION_MA = 40.0f;
  static constexpr float FRICTION_COMPENSATION_MIN_ERROR_DEG = 2.0f;
  static constexpr float FRICTION_COMPENSATION_MAX_ERROR_DEG = 10.0f;
  static constexpr float FRICTION_COMPENSATION_MAX_RATE_DEG_PER_SEC = 0.5f;
  static constexpr float NEAR_TARGET_ANGLE_DEG = 15.0f;
  static constexpr float NEAR_TARGET_MAX_CURRENT_MA = 80.0f;
  static constexpr float NEAR_TARGET_MAX_CURRENT_SLEW_MA_PER_UPDATE = 10.0f;
  static constexpr unsigned long NEAR_TARGET_COMMAND_INTERVAL_MS = 125;
  // Keep Roller I2C traffic low.  Current is written only when it changes by
  // this amount (or when zero must be sent), and no faster than 4 Hz.
  static constexpr unsigned long CURRENT_COMMAND_INTERVAL_MS = 250;
  static constexpr float CURRENT_COMMAND_MIN_CHANGE_MA = 10.0f;
  static constexpr float NEAR_TARGET_CURRENT_MIN_CHANGE_MA = 5.0f;
  // One item is read per interval, so a complete diagnostic scan takes 9 s.
  static constexpr unsigned long WHEEL_DIAGNOSTIC_INTERVAL_MS = 1500;
  static constexpr uint8_t MAX_CONSECUTIVE_DIAGNOSTIC_FAILURES = 3;

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
  uint8_t wheel_system_status() const;
  uint8_t wheel_error_code() const;
  uint8_t wheel_output_status() const;
  uint8_t wheel_stall_protection_status() const;
  bool wheel_communication_is_valid() const;
  uint8_t wheel_diagnostic_failure_count() const;
  const char *control_state_name() const;

 private:
  enum ControlState {
    CONTROL_STATE_ACCELERATING,
    CONTROL_STATE_BRAKING,
    CONTROL_STATE_SATURATED,
    CONTROL_STATE_TARGET_HOLD
  };

  static float normalize_angle_deg(float angle_deg);
  float calculate_body_pd_command(
      float estimated_yaw_deg, float yaw_rate_deg_per_sec) const;
  void update_legacy_speed_pd(AngularEstimation &estimation, Bno055 &sensor,
                              UnitRollerI2C &wheel, unsigned long now_ms);

  bool enabled_ = false;
  float target_angle_deg_ = 0.0f;
  float kp_ma_per_deg_ = 5.0f;
  float kd_ma_per_deg_per_sec_ = 3.0f;
  int32_t current_command_ = 0;
  int32_t last_sent_current_command_ = 0;
  int32_t current_readback_ = 0;
  bool current_readback_is_valid_ = false;
  int32_t wheel_speed_rpm_ = 0;
  bool wheel_readback_is_valid_ = false;
  uint8_t wheel_system_status_ = 0;
  uint8_t wheel_error_code_ = 0;
  uint8_t wheel_output_status_ = 0;
  uint8_t wheel_stall_protection_status_ = 0;
  uint8_t wheel_diagnostic_index_ = 0;
  uint8_t consecutive_diagnostic_failures_ = 0;
  bool communication_fault_latched_ = false;
  float wheel_speed_command_rpm_ = 0.0f;
  unsigned long previous_speed_step_ms_ = 0;
  unsigned long previous_wheel_command_write_ms_ = 0;
  unsigned long previous_wheel_diagnostic_ms_ = 0;
  int32_t last_sent_wheel_speed_rpm_ = 0;
  unsigned long previous_wheel_readback_ms_ = 0;
  unsigned long saturation_started_ms_ = 0;
  unsigned long settling_started_ms_ = 0;
  unsigned long target_hold_candidate_started_ms_ = 0;
  unsigned long target_hold_exit_started_ms_ = 0;
  unsigned long wheel_tracking_error_started_ms_ = 0;
  bool wheel_is_recovering_ = false;
  ControlState control_state_ = CONTROL_STATE_ACCELERATING;
};

#endif  // ADCS_CONTROL_H
