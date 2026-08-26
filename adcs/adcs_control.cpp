#include "adcs_control.h"

String AdcsControl::start(AngularEstimation &estimation, UnitRollerI2C &wheel,
                          bool &wheel_is_running, unsigned long now_ms) {
  estimation.reset(now_ms);
  wheel_speed_rpm_ = wheel.getSpeedReadbackRpm();
  wheel_speed_command_rpm_ = static_cast<float>(wheel_speed_rpm_);
  previous_speed_step_ms_ = now_ms - SPEED_COMMAND_STEP_INTERVAL_MS;
  previous_wheel_command_write_ms_ = now_ms;
  last_sent_wheel_speed_rpm_ = lroundf(wheel_speed_command_rpm_);
  previous_wheel_readback_ms_ = now_ms;
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  target_hold_candidate_started_ms_ = 0;
  target_hold_exit_started_ms_ = 0;
  wheel_tracking_error_started_ms_ = 0;
  wheel_is_recovering_ = false;
  control_state_ = CONTROL_STATE_NORMAL;
  current_command_ = 0;
  // Use the same tested start sequence as Labxx_wheel_angular_momentum-main.
  wheel.resetStalledProtect();
  wheel.start(lroundf(wheel_speed_command_rpm_),
              SPEED_MODE_MAX_CURRENT_REGISTER);
  wheel_is_running = true;
  enabled_ = true;
  return "OK START SPEED=" + String(wheel_speed_command_rpm_, 1) + " rpm";
}

String AdcsControl::stop(UnitRollerI2C &wheel, bool &wheel_is_running) {
  enabled_ = false;
  current_command_ = 0;
  current_readback_ = 0;
  wheel_speed_command_rpm_ = 0.0f;
  last_sent_wheel_speed_rpm_ = 0;
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  target_hold_candidate_started_ms_ = 0;
  target_hold_exit_started_ms_ = 0;
  wheel_tracking_error_started_ms_ = 0;
  wheel_is_recovering_ = false;
  control_state_ = CONTROL_STATE_NORMAL;
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

  // Latch the wheel speed after the attitude has remained settled. Merely
  // skipping one PD update is not sufficient: sensor noise can otherwise
  // make the accumulated speed command drift to saturation and trigger an
  // unloading manoeuvre that moves the body away from the target.
  if (control_state_ == CONTROL_STATE_TARGET_HOLD) {
    if (fabsf(angle_error_deg) > TARGET_HOLD_EXIT_ERROR_DEG) {
      if (target_hold_exit_started_ms_ == 0) {
        target_hold_exit_started_ms_ = now_ms;
      } else if (now_ms - target_hold_exit_started_ms_ >=
                 TARGET_HOLD_EXIT_CONFIRM_MS) {
        control_state_ = CONTROL_STATE_NORMAL;
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
  if (control_state_ == CONTROL_STATE_NORMAL && target_is_settled) {
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

  // Remove stored wheel momentum gradually after saturation. The yaw
  // estimate is intentionally not reset, so control resumes for the
  // remaining attitude error.
  if (control_state_ == CONTROL_STATE_UNLOADING) {
    if (now_ms - previous_speed_step_ms_ < SPEED_COMMAND_STEP_INTERVAL_MS) {
      return;
    }
    previous_speed_step_ms_ = now_ms;

    if (wheel_speed_command_rpm_ > UNLOAD_SPEED_STEP_RPM) {
      wheel_speed_command_rpm_ -= UNLOAD_SPEED_STEP_RPM;
    } else if (wheel_speed_command_rpm_ < -UNLOAD_SPEED_STEP_RPM) {
      wheel_speed_command_rpm_ += UNLOAD_SPEED_STEP_RPM;
    } else {
      wheel_speed_command_rpm_ = 0.0f;
    }
    // Batch the 250 ms control decisions into one Roller write per second.
    // The standalone wheel test is reliable because it writes only when a
    // command is entered; continuous 4 Hz writes eventually stall this bus.
    const int32_t requested_wheel_speed_rpm =
        lroundf(wheel_speed_command_rpm_);
    const bool unload_reached_zero = requested_wheel_speed_rpm == 0;
    const bool enough_change_accumulated =
        labs(static_cast<long>(requested_wheel_speed_rpm) -
             static_cast<long>(last_sent_wheel_speed_rpm_)) >=
        WHEEL_COMMAND_MIN_CHANGE_RPM;
    const bool write_interval_elapsed =
        now_ms - previous_wheel_command_write_ms_ >=
        WHEEL_COMMAND_WRITE_INTERVAL_MS;
    if ((unload_reached_zero || enough_change_accumulated) &&
        write_interval_elapsed) {
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

    if (unload_reached_zero && last_sent_wheel_speed_rpm_ == 0) {
      control_state_ = CONTROL_STATE_SETTLING;
      settling_started_ms_ = 0;
    }
    return;
  }

  if (control_state_ == CONTROL_STATE_SETTLING) {
    const bool wheel_is_stopped =
        fabsf(wheel_speed_command_rpm_) <= UNLOAD_FINISHED_SPEED_RPM;
    const bool body_is_stopped =
        fabsf(yaw_rate_deg_per_sec) <= SETTLED_YAW_RATE_DEG_PER_SEC;
    if (wheel_is_stopped && body_is_stopped) {
      if (settling_started_ms_ == 0) settling_started_ms_ = now_ms;
      if (now_ms - settling_started_ms_ >= SETTLING_CONFIRM_MS) {
        control_state_ = CONTROL_STATE_NORMAL;
        saturation_started_ms_ = 0;
        settling_started_ms_ = 0;
        previous_speed_step_ms_ = now_ms;
      }
    } else {
      settling_started_ms_ = 0;
    }
    return;
  }

  const bool pushing_positive_saturation =
      last_sent_wheel_speed_rpm_ >= UNLOAD_START_SPEED_RPM &&
      body_pd_command > CONTROL_SWITCH_THRESHOLD;
  const bool pushing_negative_saturation =
      last_sent_wheel_speed_rpm_ <= -UNLOAD_START_SPEED_RPM &&
      body_pd_command < -CONTROL_SWITCH_THRESHOLD;
  const bool unload_is_needed =
      fabsf(angle_error_deg) > UNLOAD_MIN_ERROR_DEG &&
      (pushing_positive_saturation || pushing_negative_saturation);

  if (unload_is_needed) {
    if (saturation_started_ms_ == 0) {
      saturation_started_ms_ = now_ms;
      control_state_ = CONTROL_STATE_SATURATED;
    }

    // Do not wait for the body to stop at the speed limit. If it has already
    // started moving away from the target, unload immediately; otherwise
    // confirm saturation for one second before unloading.
    const bool moving_away_from_target =
        angle_error_deg * yaw_rate_deg_per_sec < 0.0f &&
        fabsf(yaw_rate_deg_per_sec) >= MOVING_AWAY_YAW_RATE_DEG_PER_SEC;
    if (moving_away_from_target ||
        now_ms - saturation_started_ms_ >= SATURATION_CONFIRM_MS) {
      control_state_ = CONTROL_STATE_UNLOADING;
      previous_speed_step_ms_ = now_ms - SPEED_COMMAND_STEP_INTERVAL_MS;
      saturation_started_ms_ = 0;
      return;
    }
  } else {
    saturation_started_ms_ = 0;
    if (control_state_ == CONTROL_STATE_SATURATED) {
      control_state_ = CONTROL_STATE_NORMAL;
    }
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

  // Use a large speed change far from the target and a small change near it.
  // The PD magnitude also includes yaw-rate damping, so it can reduce the
  // command early when the body is already rotating quickly.
  const float absolute_pd_command = fabsf(body_pd_command);
  int32_t speed_step_rpm = 0;
  if (absolute_pd_command >= LARGE_STEP_THRESHOLD) {
    speed_step_rpm = LARGE_SPEED_STEP_RPM;
  } else if (absolute_pd_command >= MEDIUM_STEP_THRESHOLD) {
    speed_step_rpm = MEDIUM_SPEED_STEP_RPM;
  } else if (absolute_pd_command >= CONTROL_SWITCH_THRESHOLD) {
    speed_step_rpm = SMALL_SPEED_STEP_RPM;
  } else {
    return;
  }

  // Positive reported wheel RPM produces the positive measured yaw response;
  // this sign was determined from the stepped-response test.
  float requested_speed_rpm = wheel_speed_command_rpm_;
  if (body_pd_command > 0.0f) {
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
  // A new target always starts a new manoeuvre. In particular, do not keep a
  // TARGET_HOLD latch or continue unloading for the previous target.
  target_hold_candidate_started_ms_ = 0;
  target_hold_exit_started_ms_ = 0;
  saturation_started_ms_ = 0;
  settling_started_ms_ = 0;
  control_state_ = CONTROL_STATE_NORMAL;
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
bool AdcsControl::current_readback_is_valid() const { return false; }
int32_t AdcsControl::wheel_speed_rpm() const { return wheel_speed_rpm_; }
bool AdcsControl::wheel_readback_is_valid() const { return true; }
float AdcsControl::wheel_speed_command_rpm() const {
  return wheel_speed_command_rpm_;
}
int32_t AdcsControl::last_sent_wheel_speed_rpm() const {
  return last_sent_wheel_speed_rpm_;
}
const char *AdcsControl::control_state_name() const {
  if (!enabled_) return "STOPPED";
  if (wheel_is_recovering_) return "WHEEL_RECOVERY";
  switch (control_state_) {
    case CONTROL_STATE_SATURATED:
      return "SATURATED";
    case CONTROL_STATE_UNLOADING:
      return "UNLOADING";
    case CONTROL_STATE_SETTLING:
      return "SETTLING";
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

