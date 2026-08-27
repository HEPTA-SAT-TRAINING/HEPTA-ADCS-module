#include "adcs_control.h"

String AdcsControl::start(AngularEstimation &estimation, UnitRollerI2C &wheel,
                          bool &wheel_is_running, unsigned long now_ms) {
  estimation.reset(now_ms);
  wheel_speed_rpm_ = wheel.getSpeedReadbackRpm();
  wheel_speed_command_rpm_ = static_cast<float>(wheel_speed_rpm_);
  previous_speed_step_ms_ = now_ms - SPEED_COMMAND_STEP_INTERVAL_MS;
  previous_wheel_command_write_ms_ = now_ms;
  previous_wheel_diagnostic_ms_ = now_ms - WHEEL_DIAGNOSTIC_INTERVAL_MS;
  last_sent_wheel_speed_rpm_ = lroundf(wheel_speed_command_rpm_);
  previous_wheel_readback_ms_ = now_ms;
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  target_hold_candidate_started_ms_ = 0;
  target_hold_exit_started_ms_ = 0;
  wheel_tracking_error_started_ms_ = 0;
  wheel_is_recovering_ = false;
  wheel_diagnostic_index_ = 0;
  consecutive_diagnostic_failures_ = 0;
  communication_fault_latched_ = false;
  control_state_ = CONTROL_STATE_ACCELERATING;
  current_command_ = 0;
  last_sent_current_command_ = 0;
  // Current mode makes the PD output proportional to wheel torque instead of
  // accumulating it into a wheel-speed target.
  wheel.resetStalledProtect();
  wheel.setOutput(0);
  wheel.setMode(ROLLER_MODE_CURRENT);
  wheel.setCurrent(0);
  wheel.setOutput(1);
  wheel_is_running = true;
  enabled_ = true;
  return "OK START CURRENT_PD LIMIT=" +
         String(MAX_CONTROL_CURRENT_MA, 1) + " mA";
}

String AdcsControl::stop(UnitRollerI2C &wheel, bool &wheel_is_running) {
  enabled_ = false;
  wheel.setCurrent(0);
  current_command_ = 0;
  last_sent_current_command_ = 0;
  current_readback_ = 0;
  current_readback_is_valid_ = false;
  wheel_readback_is_valid_ = false;
  current_readback_ = 0;
  current_readback_is_valid_ = false;
  wheel_readback_is_valid_ = false;
  wheel_speed_command_rpm_ = 0.0f;
  last_sent_wheel_speed_rpm_ = 0;
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  target_hold_candidate_started_ms_ = 0;
  target_hold_exit_started_ms_ = 0;
  wheel_tracking_error_started_ms_ = 0;
  wheel_is_recovering_ = false;
  control_state_ = CONTROL_STATE_ACCELERATING;
  // Use the same tested stop sequence as Labxx_wheel_angular_momentum-main.
  wheel.stop();
  wheel_is_running = false;
  return "OK STOP";
}

// This definition is missing from the referenced commit even though the
// function is declared and called there. It is the PD expression intended by
// that implementation and is required for the sketch to link.
float AdcsControl::calculate_body_pd_command(
    float estimated_yaw_deg, float yaw_rate_deg_per_sec) const {
  return kp_ma_per_deg_ * error_deg(estimated_yaw_deg) -
         kd_ma_per_deg_per_sec_ * yaw_rate_deg_per_sec;
}

void AdcsControl::update(AngularEstimation &estimation, Bno055 &sensor,
                         UnitRollerI2C &wheel, unsigned long now_ms) {
  if (!enabled_) return;

  estimation.update(sensor, now_ms);
  if (communication_fault_latched_) {
    current_command_ = 0;
    control_state_ = CONTROL_STATE_SATURATED;
    return;
  }
  const float estimated_yaw_deg = estimation.yaw_deg();
  const float yaw_rate_deg_per_sec = estimation.yaw_rate_deg_per_sec();
  const float angle_error_deg = error_deg(estimated_yaw_deg);
  const float absolute_angle_error_deg = fabsf(angle_error_deg);
  const bool near_target =
      absolute_angle_error_deg <= NEAR_TARGET_ANGLE_DEG;
  const bool target_is_settled =
      absolute_angle_error_deg <= SETTLED_ANGLE_ERROR_DEG &&
      fabsf(yaw_rate_deg_per_sec) <= SETTLED_YAW_RATE_DEG_PER_SEC;

  float desired_current_ma = 0.0f;
  if (target_is_settled) {
    control_state_ = CONTROL_STATE_TARGET_HOLD;
  } else {
    desired_current_ma = constrain(
        calculate_body_pd_command(estimated_yaw_deg,
                                  yaw_rate_deg_per_sec),
        near_target ? -NEAR_TARGET_MAX_CURRENT_MA : -MAX_CONTROL_CURRENT_MA,
        near_target ? NEAR_TARGET_MAX_CURRENT_MA : MAX_CONTROL_CURRENT_MA);
    // Add only enough current to overcome static friction after the body has
    // nearly stopped. During motion the ordinary D term remains unmodified.
    if (fabsf(yaw_rate_deg_per_sec) <=
            FRICTION_COMPENSATION_MAX_RATE_DEG_PER_SEC &&
        absolute_angle_error_deg >= FRICTION_COMPENSATION_MIN_ERROR_DEG &&
        absolute_angle_error_deg <= FRICTION_COMPENSATION_MAX_ERROR_DEG &&
        fabsf(desired_current_ma) < FRICTION_COMPENSATION_MA) {
      desired_current_ma = angle_error_deg >= 0.0f
          ? FRICTION_COMPENSATION_MA
          : -FRICTION_COMPENSATION_MA;
    }
    control_state_ = desired_current_ma * angle_error_deg >= 0.0f
        ? CONTROL_STATE_ACCELERATING
        : CONTROL_STATE_BRAKING;
  }

  const unsigned long command_interval_ms = near_target
      ? NEAR_TARGET_COMMAND_INTERVAL_MS
      : CURRENT_COMMAND_INTERVAL_MS;
  if (now_ms - previous_wheel_command_write_ms_ >= command_interval_ms) {
    previous_wheel_command_write_ms_ = now_ms;

    const int32_t desired_current_register =
        lroundf(desired_current_ma * CURRENT_REGISTER_PER_MA);
    const float maximum_slew_ma = near_target
        ? NEAR_TARGET_MAX_CURRENT_SLEW_MA_PER_UPDATE
        : MAX_CURRENT_SLEW_MA_PER_UPDATE;
    const int32_t maximum_slew_register =
        lroundf(maximum_slew_ma * CURRENT_REGISTER_PER_MA);
    const int32_t current_change = constrain(
        desired_current_register - current_command_,
        -maximum_slew_register, maximum_slew_register);
    current_command_ += current_change;

    const float minimum_write_change_ma = near_target
        ? NEAR_TARGET_CURRENT_MIN_CHANGE_MA
        : CURRENT_COMMAND_MIN_CHANGE_MA;
    const int32_t minimum_write_change =
        lroundf(minimum_write_change_ma * CURRENT_REGISTER_PER_MA);
    const bool changed_enough =
        labs(static_cast<long>(current_command_ -
                               last_sent_current_command_)) >=
        minimum_write_change;
    const bool zero_must_be_sent =
        current_command_ == 0 && last_sent_current_command_ != 0;
    if (changed_enough || zero_must_be_sent) {
      wheel.setCurrent(current_command_);
      last_sent_current_command_ = current_command_;
    }
  }

  if (now_ms - previous_wheel_diagnostic_ms_ <
      WHEEL_DIAGNOSTIC_INTERVAL_MS) {
    return;
  }
  previous_wheel_diagnostic_ms_ = now_ms;
  bool diagnostic_succeeded = false;
  bool device_fault_detected = false;
  switch (wheel_diagnostic_index_) {
    case 0:
      current_readback_is_valid_ =
          wheel.getCurrentReadback(&current_readback_);
      diagnostic_succeeded = current_readback_is_valid_;
      break;
    case 1: {
      int32_t speed_rpm = 0;
      diagnostic_succeeded = wheel.getSpeedReadbackRpm(&speed_rpm) &&
          labs(static_cast<long>(speed_rpm)) <= MAX_VALID_WHEEL_READBACK_RPM;
      wheel_readback_is_valid_ = diagnostic_succeeded;
      if (diagnostic_succeeded) wheel_speed_rpm_ = speed_rpm;
      break;
    }
    case 2:
      diagnostic_succeeded = wheel.getSysStatus(&wheel_system_status_);
      device_fault_detected = diagnostic_succeeded &&
          wheel_system_status_ == 2;
      break;
    case 3:
      diagnostic_succeeded = wheel.getErrorCode(&wheel_error_code_);
      device_fault_detected = diagnostic_succeeded &&
          wheel_error_code_ != 0;
      break;
    case 4:
      diagnostic_succeeded = wheel.getOutputStatus(&wheel_output_status_);
      device_fault_detected = diagnostic_succeeded &&
          wheel_output_status_ == 0;
      break;
    default:
      diagnostic_succeeded = wheel.getStallProtection(
          &wheel_stall_protection_status_);
      break;
  }
  wheel_diagnostic_index_ = (wheel_diagnostic_index_ + 1) % 6;

  if (diagnostic_succeeded) {
    consecutive_diagnostic_failures_ = 0;
  } else if (consecutive_diagnostic_failures_ < 255) {
    ++consecutive_diagnostic_failures_;
  }

  if (device_fault_detected ||
      consecutive_diagnostic_failures_ >=
          MAX_CONSECUTIVE_DIAGNOSTIC_FAILURES) {
    current_command_ = 0;
    if (last_sent_current_command_ != 0) {
      // Make a single safe-stop attempt.  Do not keep writing to an already
      // failed bus on every control iteration.
      wheel.setCurrent(0);
      last_sent_current_command_ = 0;
    }
    control_state_ = CONTROL_STATE_SATURATED;
    communication_fault_latched_ = true;
  }
}

// Retained temporarily for comparison with earlier speed-mode experiments;
// the active controller above does not call this implementation.
void AdcsControl::update_legacy_speed_pd(
                         AngularEstimation &estimation, Bno055 &sensor,
                         UnitRollerI2C &wheel, unsigned long now_ms) {
  if (!enabled_) return;

  estimation.update(sensor, now_ms);
  // Do not read Roller feedback in the time-critical control loop. When an
  // I2C read fails, the driver can wait for several seconds; integrating the
  // gyro across that gap makes a stationary body appear to rotate by tens of
  // degrees. Speed mode is command based, so feedback is diagnostic only.
  wheel_is_recovering_ = false;
  const float estimated_yaw_deg = estimation.yaw_deg();
  const float yaw_rate_deg_per_sec = estimation.yaw_rate_deg_per_sec();
  const float angle_error_deg = error_deg(estimated_yaw_deg);
  const float body_pd_command = calculate_body_pd_command(
      estimated_yaw_deg, yaw_rate_deg_per_sec);

  // Keep the applied wheel momentum once the target is settled. Returning
  // the wheel to zero would rotate the body again.
  if (control_state_ == CONTROL_STATE_TARGET_HOLD) {
    if (fabsf(angle_error_deg) > TARGET_HOLD_EXIT_ERROR_DEG) {
      if (target_hold_exit_started_ms_ == 0) {
        target_hold_exit_started_ms_ = now_ms;
      } else if (now_ms - target_hold_exit_started_ms_ >=
                 TARGET_HOLD_EXIT_CONFIRM_MS) {
        control_state_ = CONTROL_STATE_ACCELERATING;
        target_hold_exit_started_ms_ = 0;
        target_hold_candidate_started_ms_ = 0;
        previous_speed_step_ms_ = now_ms;
      }
    } else {
      target_hold_exit_started_ms_ = 0;
    }

    if (control_state_ == CONTROL_STATE_TARGET_HOLD) return;
  }

  const bool target_is_settled =
      fabsf(angle_error_deg) <= SETTLED_ANGLE_ERROR_DEG &&
      fabsf(yaw_rate_deg_per_sec) <= SETTLED_YAW_RATE_DEG_PER_SEC;
  if (control_state_ != CONTROL_STATE_SATURATED && target_is_settled) {
    if (target_hold_candidate_started_ms_ == 0) {
      target_hold_candidate_started_ms_ = now_ms;
    } else if (now_ms - target_hold_candidate_started_ms_ >=
               TARGET_HOLD_CONFIRM_MS) {
      control_state_ = CONTROL_STATE_TARGET_HOLD;
      target_hold_candidate_started_ms_ = 0;
      target_hold_exit_started_ms_ = 0;
      saturation_started_ms_ = 0;
      // Hold the speed that is already applied to the wheel. Sending the
      // latest accumulated internal command here could create a large final
      // current step just as the attitude reaches its target.
      wheel_speed_command_rpm_ =
          static_cast<float>(last_sent_wheel_speed_rpm_);
      return;
    }
  } else {
    target_hold_candidate_started_ms_ = 0;
  }

  if (now_ms - previous_speed_step_ms_ < SPEED_COMMAND_STEP_INTERVAL_MS) {
    return;
  }
  previous_speed_step_ms_ = now_ms;

  // Once both attitude and rate are small, hold the current wheel speed.
  // Returning it to zero would create another momentum change.
  if (fabsf(angle_error_deg) < SETTLED_ANGLE_ERROR_DEG &&
      fabsf(yaw_rate_deg_per_sec) < SETTLED_YAW_RATE_DEG_PER_SEC) {
    return;
  }

  const bool moving_toward_target =
      angle_error_deg * yaw_rate_deg_per_sec > 0.0f;
  const float braking_angle_deg =
      yaw_rate_deg_per_sec * yaw_rate_deg_per_sec /
          (2.0f * BRAKING_ACCEL_DEG_PER_SEC2) +
      BRAKING_MARGIN_DEG;
  const int acceleration_direction = angle_error_deg >= 0.0f ? 1 : -1;
  const int wheel_acceleration_direction = acceleration_direction;
  const int32_t acceleration_reserve_rpm = wheel_acceleration_direction > 0
      ? MAX_WHEEL_SPEED_RPM - last_sent_wheel_speed_rpm_
      : MAX_WHEEL_SPEED_RPM + last_sent_wheel_speed_rpm_;

  if (control_state_ == CONTROL_STATE_BRAKING) {
    // Keep braking latched until the body is nearly stopped. Re-evaluating
    // the PD sign during braking can command more acceleration after a small
    // amount of noise or overshoot.
    if (fabsf(yaw_rate_deg_per_sec) <= SETTLED_YAW_RATE_DEG_PER_SEC) {
      control_state_ = CONTROL_STATE_ACCELERATING;
      return;
    }
  } else if (moving_toward_target &&
             (fabsf(yaw_rate_deg_per_sec) >=
                  MAX_MANEUVER_YAW_RATE_DEG_PER_SEC ||
              fabsf(angle_error_deg) <= braking_angle_deg ||
              acceleration_reserve_rpm <= BRAKING_RESERVE_RPM)) {
    control_state_ = CONTROL_STATE_BRAKING;
  } else {
    control_state_ = CONTROL_STATE_ACCELERATING;
  }

  // Convert the PD magnitude continuously into a wheel-speed change. The
  // state machine chooses the safe direction; these limits retain authority
  // near zero without allowing one update to make a large RPM jump.
  const float absolute_pd_command = fabsf(body_pd_command);
  if (absolute_pd_command < CONTROL_SWITCH_THRESHOLD &&
      control_state_ != CONTROL_STATE_BRAKING) {
    return;
  }
  const float speed_step_rpm = constrain(
      absolute_pd_command * PD_TO_SPEED_STEP_RPM,
      MIN_SPEED_STEP_RPM, MAX_SPEED_STEP_RPM);

  // The latest response shows that positive wheel RPM produces positive body
  // yaw. During braking, change wheel speed opposite to body rotation.
  int command_direction = wheel_acceleration_direction;
  if (control_state_ == CONTROL_STATE_BRAKING) {
    command_direction = yaw_rate_deg_per_sec >= 0.0f ? -1 : 1;
  }

  const bool command_would_saturate =
      (command_direction > 0 &&
       last_sent_wheel_speed_rpm_ >= MAX_WHEEL_SPEED_RPM) ||
      (command_direction < 0 &&
       last_sent_wheel_speed_rpm_ <= -MAX_WHEEL_SPEED_RPM);
  if (command_would_saturate) {
    // Do not automatically unload: changing wheel momentum while saturated
    // would rotate the body away from the target. Hold and report saturation.
    control_state_ = CONTROL_STATE_SATURATED;
    wheel_speed_command_rpm_ = static_cast<float>(last_sent_wheel_speed_rpm_);
    return;
  }

  float requested_speed_rpm = wheel_speed_command_rpm_;
  if (command_direction > 0) {
    requested_speed_rpm += speed_step_rpm;
  } else {
    requested_speed_rpm -= speed_step_rpm;
  }

  // Anti-windup for the batched wheel command: do not let the internal
  // command run hundreds of RPM ahead of the value actually sent. Otherwise
  // it can falsely reach saturation and start unloading while the physical
  // wheel is still at a modest speed.
  const float minimum_command_rpm = constrain(
      static_cast<float>(last_sent_wheel_speed_rpm_ -
                         WHEEL_COMMAND_MAX_STEP_RPM),
      -static_cast<float>(MAX_WHEEL_SPEED_RPM),
      static_cast<float>(MAX_WHEEL_SPEED_RPM));
  const float maximum_command_rpm = constrain(
      static_cast<float>(last_sent_wheel_speed_rpm_ +
                         WHEEL_COMMAND_MAX_STEP_RPM),
      -static_cast<float>(MAX_WHEEL_SPEED_RPM),
      static_cast<float>(MAX_WHEEL_SPEED_RPM));
  wheel_speed_command_rpm_ =
      constrain(requested_speed_rpm, minimum_command_rpm,
                maximum_command_rpm);
  const int32_t requested_wheel_speed_rpm =
      lroundf(wheel_speed_command_rpm_);
  const bool enough_change_accumulated =
      labs(static_cast<long>(requested_wheel_speed_rpm) -
           static_cast<long>(last_sent_wheel_speed_rpm_)) >=
      WHEEL_COMMAND_MIN_CHANGE_RPM;
  const bool write_interval_elapsed =
      now_ms - previous_wheel_command_write_ms_ >=
      WHEEL_COMMAND_WRITE_INTERVAL_MS;
  if (enough_change_accumulated && write_interval_elapsed) {
    const int32_t remaining_change_rpm =
        requested_wheel_speed_rpm - last_sent_wheel_speed_rpm_;
    const int32_t limited_change_rpm =
        constrain(remaining_change_rpm, -WHEEL_COMMAND_MAX_STEP_RPM,
                  WHEEL_COMMAND_MAX_STEP_RPM);
    const int32_t next_sent_wheel_speed_rpm =
        last_sent_wheel_speed_rpm_ + limited_change_rpm;
    wheel.setSpeedRpm(next_sent_wheel_speed_rpm);
    last_sent_wheel_speed_rpm_ = next_sent_wheel_speed_rpm;
    previous_wheel_command_write_ms_ = now_ms;
  }
}

void AdcsControl::set_target_angle_deg(float target_angle_deg) {
  target_angle_deg_ = normalize_angle_deg(target_angle_deg);
  // A new target always starts a new discrete manoeuvre.
  target_hold_candidate_started_ms_ = 0;
  target_hold_exit_started_ms_ = 0;
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  control_state_ = CONTROL_STATE_ACCELERATING;
}

bool AdcsControl::set_proportional_gain(float kp_ma_per_deg) {
  if (!isfinite(kp_ma_per_deg) || kp_ma_per_deg < 0.0f ||
      kp_ma_per_deg > MAX_KP_MA_PER_DEG) {
    return false;
  }
  kp_ma_per_deg_ = kp_ma_per_deg;
  return true;
}

bool AdcsControl::set_derivative_gain(float kd_ma_per_deg_per_sec) {
  if (!isfinite(kd_ma_per_deg_per_sec) || kd_ma_per_deg_per_sec < 0.0f ||
      kd_ma_per_deg_per_sec > MAX_KD_MA_PER_DEG_PER_SEC) {
    return false;
  }
  kd_ma_per_deg_per_sec_ = kd_ma_per_deg_per_sec;
  return true;
}

bool AdcsControl::is_enabled() const { return enabled_; }
float AdcsControl::target_angle_deg() const { return target_angle_deg_; }
float AdcsControl::proportional_gain_ma_per_deg() const {
  return kp_ma_per_deg_;
}
float AdcsControl::derivative_gain_ma_per_deg_per_sec() const {
  return kd_ma_per_deg_per_sec_;
}
float AdcsControl::error_deg(float estimated_yaw_deg) const {
  return normalize_angle_deg(target_angle_deg_ - estimated_yaw_deg);
}
float AdcsControl::current_command_ma() const {
  return current_command_ / CURRENT_REGISTER_PER_MA;
}
float AdcsControl::current_readback_ma() const {
  return current_readback_ / CURRENT_REGISTER_PER_MA;
}
float AdcsControl::friction_compensation_ma() const { return 0.0f; }
bool AdcsControl::current_readback_is_valid() const {
  return current_readback_is_valid_;
}
int32_t AdcsControl::wheel_speed_rpm() const { return wheel_speed_rpm_; }
bool AdcsControl::wheel_readback_is_valid() const {
  return wheel_readback_is_valid_;
}
float AdcsControl::wheel_speed_command_rpm() const {
  return wheel_speed_command_rpm_;
}
int32_t AdcsControl::last_sent_wheel_speed_rpm() const {
  return last_sent_wheel_speed_rpm_;
}
uint8_t AdcsControl::wheel_system_status() const {
  return wheel_system_status_;
}
uint8_t AdcsControl::wheel_error_code() const { return wheel_error_code_; }
uint8_t AdcsControl::wheel_output_status() const {
  return wheel_output_status_;
}
uint8_t AdcsControl::wheel_stall_protection_status() const {
  return wheel_stall_protection_status_;
}
bool AdcsControl::wheel_communication_is_valid() const {
  return !communication_fault_latched_;
}
uint8_t AdcsControl::wheel_diagnostic_failure_count() const {
  return consecutive_diagnostic_failures_;
}
const char *AdcsControl::control_state_name() const {
  if (!enabled_) return "STOPPED";
  if (wheel_is_recovering_) return "WHEEL_RECOVERY";
  switch (control_state_) {
    case CONTROL_STATE_ACCELERATING:
      return "ACCELERATING";
    case CONTROL_STATE_BRAKING:
      return "BRAKING";
    case CONTROL_STATE_SATURATED:
      return "SATURATED";
    case CONTROL_STATE_TARGET_HOLD:
      return "TARGET_HOLD";
    default:
      return "CONTROL";
  }
}

float AdcsControl::normalize_angle_deg(float angle_deg) {
  while (angle_deg >= 180.0f) angle_deg -= 360.0f;
  while (angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

