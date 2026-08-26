#include <Arduino.h>
#include "../adcs/angular_estimation.h"
#include "../adcs/adcs_control.h"
#include "radio_commands.h"
#include "../../com/hepta_com.h"
#include "../../drv/imu9axis_bno055.h"
#include "../../drv/unit_rolleri2c.hpp"
#include "../../eps/hepta_eps.h"

extern HeptaCom com;
extern HeptaEps eps;
extern Bno055 sensor;
extern UnitRollerI2C wheel;
extern AngularEstimation angular_estimation;
extern AdcsControl adcs_control;
extern String radioCommandBuffer;
extern bool wheelIsConnected, wheelIsRunning, is_telemetry_enabled;
extern unsigned long last_control_update_ms, last_telemetry_ms;
extern float gyro_bias_z_deg_per_sec;
extern float measured_gyro_bias_z_deg_per_sec;
extern bool measured_gyro_bias_is_valid;
extern bool gyro_bias_correction_enabled;
extern bool magnetic_calibration_active, magnetic_calibration_enabled;
extern unsigned long magnetic_calibration_started_ms;
extern unsigned long magnetic_calibration_previous_sample_ms;
extern unsigned long magnetic_calibration_previous_report_ms;
extern size_t magnetic_calibration_sample_count;
extern float magnetic_calibration_min_x_ut, magnetic_calibration_max_x_ut;
extern float magnetic_calibration_min_y_ut, magnetic_calibration_max_y_ut;
extern float magnetic_offset_x_ut, magnetic_offset_y_ut;
extern float magnetic_scale_x, magnetic_scale_y;
constexpr size_t COMMAND_BUFFER_SIZE = 64;
constexpr unsigned long COMMAND_END_TIMEOUT_MS = 100;
constexpr size_t GYRO_BIAS_SAMPLE_COUNT = 500;
constexpr unsigned long GYRO_BIAS_SAMPLE_INTERVAL_MS = 10;
constexpr float GYRO_BIAS_MAX_STDDEV_DEG_PER_SEC = 0.20f;
constexpr unsigned long MAG_CALIBRATION_DURATION_MS = 30000;
constexpr unsigned long MAG_CALIBRATION_SAMPLE_INTERVAL_MS = 50;
constexpr unsigned long MAG_CALIBRATION_REPORT_INTERVAL_MS = 5000;
constexpr float MAG_CALIBRATION_MIN_AXIS_SPAN_UT = 20.0f;
unsigned long lastcommandcharactertimems = 0;

void send_message(const String &message) {
  com.send_text(message);
  com.send_text("\r\n");
  Serial.println(message);
}
void normalize_command(String &command) { command.trim(); command.toLowerCase(); }
void enable_angular_velocity_output() { is_telemetry_enabled = true; send_message("OK TELEMETRY ON"); }
void disable_angular_velocity_output() { is_telemetry_enabled = false; send_message("OK TELEMETRY OFF"); }

bool ensure_wheel_is_connected() {
  if (wheelIsConnected) return true;
  send_message("INFO WHEEL_RECONNECTING");
  if (!connect_wheel(wheel, wheelIsConnected, WHEEL_STARTUP_TIMEOUT_MS, WHEEL_RETRY_INTERVAL_MS)) {
    send_message("ERR WHEEL_DISCONNECTED"); return false;
  }
  send_message("I2C WHEEL TRUE"); return true;
}

static bool parse_float(const String &text, float &value) {
  if (text.length() == 0) return false;
  char *endPointer = nullptr;
  value = strtof(text.c_str(), &endPointer);
  return endPointer != text.c_str() && *endPointer == '\0' && isfinite(value);
}

void execute_start_command() {
  if (magnetic_calibration_active) {
    send_message("ERR START MAGCAL_ACTIVE");
    return;
  }
  const unsigned long nowMs = millis();
  const String result =
      adcs_control.start(angular_estimation, wheel, wheelIsRunning, nowMs);
  last_control_update_ms = nowMs;
  if (!adcs_control.is_enabled()) {
    send_message(result);
    return;
  }
  send_message(result +
               " STEPPED_SPEED_PD=ON AUTO_UNLOAD=ON YAW_ZEROED");
}

void execute_stop_command() {
  adcs_control.stop(wheel, wheelIsRunning);

  // A stop command must not report success after a lost I2C write. Reconnect
  // forces the Roller's tested configure sequence (output off, speed zero),
  // then send stop once more before acknowledging the command.
  wheelIsConnected = wheel.connect(1000, 100);
  if (!wheelIsConnected) {
    send_message("ERR STOP_UNCONFIRMED WHEEL_DISCONNECTED");
    return;
  }
  wheel.stop();
  wheelIsConnected = wheel.isConnected();
  if (!wheelIsConnected) {
    send_message("ERR STOP_UNCONFIRMED I2C_WRITE_FAILED");
    return;
  }
  send_message("OK STOP CONFIRMED");
}

void execute_set_target_angle_command(const String &text) {
  float value;
  if (!parse_float(text, value)) { send_message("ERR USAGE t<yaw_deg>"); return; }
  adcs_control.set_target_angle_deg(value);
  send_message("OK TARGET_YAW " + String(adcs_control.target_angle_deg(), 2) + " deg");
}

void execute_set_kp_command(const String &text) {
  float value;
  if (!parse_float(text, value) || !adcs_control.set_proportional_gain(value)) {
    send_message("ERR KP_RANGE: kp0..kp1000 mA/deg"); return;
  }
  send_message("OK Kp " + String(value, 3) + " mA/deg");
}

void execute_set_kd_command(const String &text) {
  float value;
  if (!parse_float(text, value) || !adcs_control.set_derivative_gain(value)) {
    send_message("ERR KD_RANGE: kd0..kd1000 mA/(deg/s)"); return;
  }
  send_message("OK Kd " + String(value, 3) + " mA/(deg/s)");
}

void execute_gyro_bias_calibration_command() {
  if (magnetic_calibration_active) {
    send_message("ERR BIASCAL MAGCAL_ACTIVE");
    return;
  }
  if (adcs_control.is_enabled() || wheelIsRunning) {
    send_message("ERR BIASCAL STOP_CONTROL_AND_WHEEL_FIRST");
    return;
  }

  measured_gyro_bias_is_valid = false;
  send_message("INFO BIASCAL START KEEP_STILL 5SEC");

  // Welford's algorithm avoids loss of precision while calculating variance.
  float mean_z = 0.0f;
  float sum_squared_difference = 0.0f;
  size_t valid_samples = 0;
  for (size_t sample = 0; sample < GYRO_BIAS_SAMPLE_COUNT; ++sample) {
    float gyro_x, gyro_y, gyro_z;
    if (!sensor.sen_gyro(&gyro_x, &gyro_y, &gyro_z)) {
      send_message("ERR BIASCAL SENSOR_READ_FAILED");
      return;
    }

    ++valid_samples;
    const float difference = gyro_z - mean_z;
    mean_z += difference / valid_samples;
    const float difference_after_update = gyro_z - mean_z;
    sum_squared_difference += difference * difference_after_update;
    delay(GYRO_BIAS_SAMPLE_INTERVAL_MS);
  }

  const float variance = valid_samples > 1
      ? sum_squared_difference / (valid_samples - 1)
      : 0.0f;
  const float standard_deviation = sqrtf(variance);
  if (!isfinite(mean_z) || !isfinite(standard_deviation) ||
      standard_deviation > GYRO_BIAS_MAX_STDDEV_DEG_PER_SEC) {
    send_message("ERR BIASCAL MOTION_DETECTED STDDEV=" +
                 String(standard_deviation, 4));
    return;
  }

  measured_gyro_bias_z_deg_per_sec = mean_z;
  measured_gyro_bias_is_valid = true;
  send_message("OK BIASCAL BIAS_Z=" + String(mean_z, 4) +
               " STDDEV=" + String(standard_deviation, 4) +
               " USE biassave");
}

void execute_gyro_bias_save_command() {
  if (!measured_gyro_bias_is_valid) {
    send_message("ERR BIASSAVE RUN biascal FIRST");
    return;
  }

  gyro_bias_z_deg_per_sec = measured_gyro_bias_z_deg_per_sec;
  gyro_bias_correction_enabled = true;
  angular_estimation.reset(millis());
  send_message("OK BIASSAVE RAM_ONLY BIAS_Z=" +
               String(gyro_bias_z_deg_per_sec, 4));
}

void execute_magnetic_calibration_command() {
  if (adcs_control.is_enabled() || wheelIsRunning) {
    send_message("ERR MAGCAL STOP_CONTROL_AND_WHEEL_FIRST");
    return;
  }
  if (magnetic_calibration_active) {
    send_message("ERR MAGCAL ALREADY_ACTIVE");
    return;
  }

  magnetic_calibration_active = true;
  magnetic_calibration_started_ms = millis();
  magnetic_calibration_previous_sample_ms =
      magnetic_calibration_started_ms - MAG_CALIBRATION_SAMPLE_INTERVAL_MS;
  magnetic_calibration_previous_report_ms = magnetic_calibration_started_ms;
  magnetic_calibration_sample_count = 0;
  magnetic_calibration_min_x_ut = 1.0e9f;
  magnetic_calibration_max_x_ut = -1.0e9f;
  magnetic_calibration_min_y_ut = 1.0e9f;
  magnetic_calibration_max_y_ut = -1.0e9f;
  send_message("INFO MAGCAL START ROTATE_TURNTABLE_360_DEG FOR_30SEC");
}

void execute_magnetic_calibration_status_command() {
  String message = "MAGCAL ACTIVE=";
  message += magnetic_calibration_active ? "1" : "0";
  message += magnetic_calibration_enabled ? " APPLIED=1" : " APPLIED=0";
  message += " SAMPLES=" + String(magnetic_calibration_sample_count);
  message += " OFFSET_X=" + String(magnetic_offset_x_ut, 3);
  message += " OFFSET_Y=" + String(magnetic_offset_y_ut, 3);
  message += " SCALE_X=" + String(magnetic_scale_x, 4);
  message += " SCALE_Y=" + String(magnetic_scale_y, 4);
  send_message(message);
}

void process_magnetic_calibration(unsigned long now_ms) {
  if (!magnetic_calibration_active ||
      now_ms - magnetic_calibration_previous_sample_ms <
          MAG_CALIBRATION_SAMPLE_INTERVAL_MS) return;
  magnetic_calibration_previous_sample_ms = now_ms;

  float magnetic_x_ut, magnetic_y_ut, magnetic_z_ut;
  if (!sensor.sen_mag(&magnetic_x_ut, &magnetic_y_ut, &magnetic_z_ut)) {
    magnetic_calibration_active = false;
    send_message("ERR MAGCAL SENSOR_READ_FAILED");
    return;
  }

  magnetic_calibration_min_x_ut = min(magnetic_calibration_min_x_ut, magnetic_x_ut);
  magnetic_calibration_max_x_ut = max(magnetic_calibration_max_x_ut, magnetic_x_ut);
  magnetic_calibration_min_y_ut = min(magnetic_calibration_min_y_ut, magnetic_y_ut);
  magnetic_calibration_max_y_ut = max(magnetic_calibration_max_y_ut, magnetic_y_ut);
  ++magnetic_calibration_sample_count;

  const unsigned long elapsed_ms = now_ms - magnetic_calibration_started_ms;
  if (now_ms - magnetic_calibration_previous_report_ms >=
      MAG_CALIBRATION_REPORT_INTERVAL_MS) {
    magnetic_calibration_previous_report_ms = now_ms;
    const unsigned long capped_elapsed_ms =
        min(elapsed_ms, MAG_CALIBRATION_DURATION_MS);
    const unsigned long remaining_seconds =
        (MAG_CALIBRATION_DURATION_MS - capped_elapsed_ms + 999) / 1000;
    send_message("INFO MAGCAL ROTATING REMAIN=" +
                 String(remaining_seconds) + "SEC");
  }
  if (elapsed_ms < MAG_CALIBRATION_DURATION_MS) return;

  magnetic_calibration_active = false;
  const float range_x =
      (magnetic_calibration_max_x_ut - magnetic_calibration_min_x_ut) * 0.5f;
  const float range_y =
      (magnetic_calibration_max_y_ut - magnetic_calibration_min_y_ut) * 0.5f;
  if (!isfinite(range_x) || !isfinite(range_y) ||
      range_x * 2.0f < MAG_CALIBRATION_MIN_AXIS_SPAN_UT ||
      range_y * 2.0f < MAG_CALIBRATION_MIN_AXIS_SPAN_UT) {
    send_message("ERR MAGCAL INSUFFICIENT_ROTATION SPAN_X=" +
                 String(range_x * 2.0f, 2) + " SPAN_Y=" +
                 String(range_y * 2.0f, 2));
    return;
  }

  const float average_range = (range_x + range_y) * 0.5f;
  magnetic_offset_x_ut =
      (magnetic_calibration_max_x_ut + magnetic_calibration_min_x_ut) * 0.5f;
  magnetic_offset_y_ut =
      (magnetic_calibration_max_y_ut + magnetic_calibration_min_y_ut) * 0.5f;
  magnetic_scale_x = average_range / range_x;
  magnetic_scale_y = average_range / range_y;
  magnetic_calibration_enabled = true;
  angular_estimation.reset(now_ms);
  send_message("OK MAGCAL RAM_ONLY OFFSET_X=" +
               String(magnetic_offset_x_ut, 3) + " OFFSET_Y=" +
               String(magnetic_offset_y_ut, 3) + " SCALE_X=" +
               String(magnetic_scale_x, 4) + " SCALE_Y=" +
               String(magnetic_scale_y, 4));
}

void execute_status_command() {
  const float estimatedYawDeg = angular_estimation.yaw_deg();
  String message = "STATUS CONTROL=";
  message.reserve(240);
  message += adcs_control.is_enabled() ? "ON" : "OFF";
  message += " MODE=" + String(adcs_control.control_state_name());
  message += " YAW=" + String(estimatedYawDeg, 2) + " deg";
  message += " TARGET_YAW=" + String(adcs_control.target_angle_deg(), 2) + " deg";
  message += " ERROR=" + String(angular_estimation.error_deg(
      adcs_control.target_angle_deg()), 2) + " deg";
  message += " Kp=" + String(adcs_control.proportional_gain_ma_per_deg(), 3) + " mA/deg";
  message += " Kd=" + String(adcs_control.derivative_gain_ma_per_deg_per_sec(), 3) + " mA/(deg/s)";
  message += " WHEEL_RPM=" + String(adcs_control.wheel_speed_rpm());
  message += adcs_control.wheel_readback_is_valid()
                 ? " WHEELOK=1" : " WHEELOK=0";
  message += " CURRENT_CMD=" + String(adcs_control.current_command_ma(), 1) + " mA";
  message += " CURRENT_FB=" + String(adcs_control.current_readback_ma(), 1) + " mA";
  message += adcs_control.current_readback_is_valid()
                 ? " CURRENTOK=1" : " CURRENTOK=0";
  message += " GYRO_BIAS=" + String(gyro_bias_z_deg_per_sec, 4) + " deg/s";
  message += gyro_bias_correction_enabled ? " BIAS_CORR=ON" : " BIAS_CORR=OFF";
  message += magnetic_calibration_enabled ? " MAG_CAL=ON" : " MAG_CAL=OFF";
  send_message(message);
}

void send_telemetry() {
  const float estimatedYawDeg = angular_estimation.yaw_deg();
  // Keep the radio line compact. SoftwareSerial cannot receive while it is
  // transmitting, so shorter lines make commands responsive at 38400 baud.
  String message;
  message.reserve(140);
  message = "ATT ms=" + String(millis());
  message += " yaw=" + String(estimatedYawDeg, 2);
  message += " rate=" + String(angular_estimation.yaw_rate_deg_per_sec(), 2);
  message += " mag=" + String(angular_estimation.magnetic_yaw_deg(), 2);
  message += angular_estimation.magnetometer_is_valid() ? " magok=1" : " magok=0";
  message += " mode=" + String(adcs_control.control_state_name());
  message += " target=" + String(adcs_control.target_angle_deg(), 2);
  message += " error=" + String(angular_estimation.error_deg(
      adcs_control.target_angle_deg()), 2);
  message += " rpm=" + String(adcs_control.wheel_speed_rpm());
  message += adcs_control.wheel_readback_is_valid()
                 ? " wheelok=1" : " wheelok=0";
  message += " cur=" + String(adcs_control.current_command_ma(), 1);
  message += " curfb=" + String(adcs_control.current_readback_ma(), 1);
  message += adcs_control.current_readback_is_valid()
                 ? " currentok=1" : " currentok=0";
  message += " boost=" + String(adcs_control.friction_compensation_ma(), 1);
  message += " voltage=" + String(eps.get_battery_voltage(), 2);
  send_message(message);
}

static void advance_periodic_timestamp(unsigned long now_ms,
                                       unsigned long interval_ms,
                                       unsigned long &last_update_ms) {
  const unsigned long elapsed_ms = now_ms - last_update_ms;
  // Preserve the normal time grid after a small delay. After missing multiple
  // periods, skip stale updates instead of emitting them in a burst.
  last_update_ms = elapsed_ms < 2 * interval_ms
                       ? last_update_ms + interval_ms
                       : now_ms;
}

void process_telemetry(
    unsigned long now_ms, unsigned long telemetry_interval_ms) {
  if (is_telemetry_enabled && adcs_control.is_enabled() &&
      now_ms - last_telemetry_ms >= telemetry_interval_ms) {
    send_telemetry();
    advance_periodic_timestamp(
        now_ms, telemetry_interval_ms, last_telemetry_ms);
  }
}

void send_command_error() { send_message("ERR COMMANDS: t<yaw_deg>, kp<mA/deg>, kd<mA/(deg/s)>, biascal, biassave, magcal, magcal?, a, s, h, v, p"); }
static bool is_single_character_command(char c) {
  c = static_cast<char>(tolower(c));
  return c == 'a' || c == 's' || c == 'h' || c == 'v' || c == 'p' || c == '?';
}

void receive_radio_commands(CommandHandler handler) {
  String received = com.get_text();
  while (Serial.available() > 0) {
    received += static_cast<char>(Serial.read());
  }

  for (unsigned int i = 0; i < received.length(); ++i) {
    const char current_character = received.charAt(i);
    const bool is_gain_command =
        radioCommandBuffer.equalsIgnoreCase("k") &&
        (static_cast<char>(tolower(current_character)) == 'p' ||
         static_cast<char>(tolower(current_character)) == 'd');

    if (is_gain_command) {
      radioCommandBuffer += current_character;
      lastcommandcharactertimems = millis();
    }

    else if (current_character == '\r' || current_character == '\n') {
      if (radioCommandBuffer.length() > 0) {
        handler(radioCommandBuffer);
        radioCommandBuffer = "";
      }
    }

    else if (radioCommandBuffer.length() < COMMAND_BUFFER_SIZE - 1) {
      radioCommandBuffer += current_character;
      lastcommandcharactertimems = millis();
    }

    else {
      radioCommandBuffer = ""; send_message("ERR COMMAND_TOO_LONG");
    }
  }

  if (radioCommandBuffer.length() > 0 &&
      millis() - lastcommandcharactertimems >= COMMAND_END_TIMEOUT_MS) {
    handler(radioCommandBuffer);
    radioCommandBuffer = "";
  }
}
