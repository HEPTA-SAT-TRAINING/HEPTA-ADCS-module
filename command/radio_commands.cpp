#include "radio_commands.h"
#include <stdlib.h>
#include "../adcs/angular_estimation.h"
#include "../../src/HeptaSat.h"
#include "../../src/drv/unit_rolleri2c.hpp"

namespace {
constexpr int32_t MAX_RPM = 3000, SPEED_SCALE = 100, MAX_CURRENT = 100000;
constexpr float CURRENT_SCALE = 100.0f;
constexpr unsigned long TELEMETRY_MS = 200, COMMAND_TIMEOUT_MS = 100;
constexpr size_t GYRO_BIAS_SAMPLE_COUNT = 500;
constexpr unsigned long GYRO_BIAS_SAMPLE_INTERVAL_MS = 10;
constexpr float GYRO_BIAS_MAX_STDDEV_DEG_PER_SEC = 0.20f;
constexpr unsigned long MAG_CALIBRATION_DURATION_MS = 30000;
constexpr unsigned long MAG_CALIBRATION_SAMPLE_INTERVAL_MS = 50;
constexpr unsigned long MAG_CALIBRATION_REPORT_INTERVAL_MS = 5000;
constexpr float MAG_CALIBRATION_MIN_AXIS_SPAN_UT = 20.0f;
HeptaCom *radio;
HeptaEps *power;
HeptaSensor *imu;
UnitRollerI2C *reaction_wheel;
AngularEstimation *attitude_estimator;
String buffer;
bool wheel_connected = false, telemetry = false;
bool wheel_running = false;
float gyro_bias_z_deg_per_sec = 0.0f;
float measured_gyro_bias_z_deg_per_sec = 0.0f;
bool measured_gyro_bias_is_valid = false;
bool gyro_bias_correction_enabled = false;
bool magnetic_calibration_active = false;
bool magnetic_calibration_enabled = false;
unsigned long magnetic_calibration_started_ms = 0;
unsigned long magnetic_calibration_previous_sample_ms = 0;
unsigned long magnetic_calibration_previous_report_ms = 0;
size_t magnetic_calibration_sample_count = 0;
float magnetic_calibration_min_x_ut = 0.0f, magnetic_calibration_max_x_ut = 0.0f;
float magnetic_calibration_min_y_ut = 0.0f, magnetic_calibration_max_y_ut = 0.0f;
float magnetic_offset_x_ut = 0.0f, magnetic_offset_y_ut = 0.0f;
float magnetic_scale_x = 1.0f, magnetic_scale_y = 1.0f;
unsigned long last_character_ms = 0, last_telemetry_ms = 0;

void send(const String &text) {
  Serial.println(text);
  radio->send_text(text + "\r\n");
}

bool parse_rpm(const String &text, int32_t &rpm) {
  char *end = nullptr;
  const long value = strtol(text.c_str(), &end, 10);
  if (!text.length() || end == text.c_str() || *end || value < -MAX_RPM || value > MAX_RPM)
    return false;
  rpm = value;
  return true;
}

void append(const String &text, CommandHandler command_handler) {
  for (unsigned int i = 0; i < text.length(); ++i) {
    const char c = text[i];
    if (c == '\r' || c == '\n') { if (buffer.length()) command_handler(buffer); buffer = ""; }
    else if (buffer.length() < 31) { buffer += c; last_character_ms = millis(); }
    else { buffer = ""; send("ERR COMMAND TOO LONG"); }
  }
}

void output_telemetry() {
  String line;
  line.reserve(220);
  line += "EstimationYaw_deg:" + String(attitude_estimator->yaw_deg(), 2);
  line += "\tGyroX_dps:" + String(attitude_estimator->gyro_x(), 2);
  line += "\tGyroY_dps:" + String(attitude_estimator->gyro_y(), 2);
  line += "\tGyroZ_dps:" + String(attitude_estimator->gyro_z(), 2);
  line += "\tMagX_uT:" + String(attitude_estimator->mag_x(), 2);
  line += "\tMagY_uT:" + String(attitude_estimator->mag_y(), 2);
  line += "\tMagZ_uT:" + String(attitude_estimator->mag_z(), 2);
  line += "\tWheelSpeed_RPM:" + String(wheel_connected ? reaction_wheel->getSpeedReadback() / SPEED_SCALE : 0);
  line += "\tWheelCurrent_mA:" + String(wheel_connected ? reaction_wheel->getCurrentReadback() / CURRENT_SCALE : 0, 2);
  line += "\tSatelliteVoltage_V:" + String(power->get_bus_voltage(), 3);
  send(line);
}
}

void begin_adcs_module(HeptaCom &com, HeptaEps &eps, HeptaSensor &sensor,
                       UnitRollerI2C &wheel, AngularEstimation &estimation) {
  radio = &com; power = &eps; imu = &sensor; reaction_wheel = &wheel;
  attitude_estimator = &estimation;
  Serial.begin(115200);
  radio->begin();
  power->init(); power->switch_3V3_on(); delay(1000);
  const bool imu_ok = imu->begin();
  Wire1.setSDA(6); Wire1.setSCL(7);
  wheel_connected = reaction_wheel->begin(&Wire1, I2C_ADDR, 100000);
  if (wheel_connected) reaction_wheel->setOutput(0);
  attitude_estimator->reset(millis());
  send(imu_ok ? "IMU TRUE" : "IMU FALSE");
  send(wheel_connected ? "I2C WHEEL TRUE" : "I2C WHEEL FALSE");
  send("READY: est_j, est_m, est_f, biascal, biassave, magcal, magcal?, t, r<rpm>, s");
}

void normalize_command(String &command) {
  command.trim();
  command.toLowerCase();
}

void execute_estimation_mode_command(AngularEstimation::Mode mode) {
  attitude_estimator->set_mode(mode, millis());
  send("OK ESTIMATION " + String(attitude_estimator->mode_name()));
}

void enable_telemetry() {
  telemetry = true;
  last_telemetry_ms = millis() - TELEMETRY_MS;
  send("OK TELEMETRY ON");
}

void execute_set_wheel_speed_command(const String &speed_text) {
  int32_t rpm;
  if (!parse_rpm(speed_text, rpm)) send("ERR USAGE r<rpm> (-3000..3000)");
  else if (!wheel_connected) send("ERR WHEEL DISCONNECTED");
  else {
    reaction_wheel->setMode(ROLLER_MODE_SPEED);
    reaction_wheel->setSpeedMaxCurrent(MAX_CURRENT);
    reaction_wheel->setSpeed(rpm * SPEED_SCALE);
    reaction_wheel->setOutput(1);
    wheel_running = rpm != 0;
    send("OK WHEEL RUN " + String(rpm) + " RPM");
  }
}

void execute_wheel_stop_command() {
  if (!wheel_connected) send("ERR WHEEL DISCONNECTED");
  else {
    reaction_wheel->setSpeed(0);
    reaction_wheel->setOutput(0);
    wheel_running = false;
    send("OK WHEEL STOP");
  }
}

void execute_gyro_bias_calibration_command() {
  if (magnetic_calibration_active) {
    send("ERR BIASCAL MAGCAL_ACTIVE");
    return;
  }
  if (wheel_running) {
    send("ERR BIASCAL STOP_WHEEL_FIRST");
    return;
  }

  measured_gyro_bias_is_valid = false;
  send("INFO BIASCAL START KEEP_STILL 5SEC");
  float mean_z = 0.0f;
  float sum_squared_difference = 0.0f;
  size_t valid_samples = 0;
  for (size_t sample = 0; sample < GYRO_BIAS_SAMPLE_COUNT; ++sample) {
    float gyro_x, gyro_y, gyro_z;
    if (!imu->get_gyro(&gyro_x, &gyro_y, &gyro_z)) {
      send("ERR BIASCAL SENSOR_READ_FAILED");
      return;
    }
    ++valid_samples;
    const float difference = gyro_z - mean_z;
    mean_z += difference / valid_samples;
    sum_squared_difference += difference * (gyro_z - mean_z);
    delay(GYRO_BIAS_SAMPLE_INTERVAL_MS);
  }

  const float variance = valid_samples > 1
      ? sum_squared_difference / (valid_samples - 1) : 0.0f;
  const float standard_deviation = sqrtf(variance);
  if (!isfinite(mean_z) || !isfinite(standard_deviation) ||
      standard_deviation > GYRO_BIAS_MAX_STDDEV_DEG_PER_SEC) {
    send("ERR BIASCAL MOTION_DETECTED STDDEV=" + String(standard_deviation, 4));
    return;
  }
  measured_gyro_bias_z_deg_per_sec = mean_z;
  measured_gyro_bias_is_valid = true;
  send("OK BIASCAL BIAS_Z=" + String(mean_z, 4) +
       " STDDEV=" + String(standard_deviation, 4) + " USE biassave");
}

void execute_gyro_bias_save_command() {
  if (!measured_gyro_bias_is_valid) {
    send("ERR BIASSAVE RUN biascal FIRST");
    return;
  }
  gyro_bias_z_deg_per_sec = measured_gyro_bias_z_deg_per_sec;
  gyro_bias_correction_enabled = true;
  attitude_estimator->reset(millis());
  send("OK BIASSAVE RAM_ONLY BIAS_Z=" + String(gyro_bias_z_deg_per_sec, 4));
}

void execute_magnetic_calibration_command() {
  if (wheel_running) {
    send("ERR MAGCAL STOP_WHEEL_FIRST");
    return;
  }
  if (magnetic_calibration_active) {
    send("ERR MAGCAL ALREADY_ACTIVE");
    return;
  }
  magnetic_calibration_active = true;
  magnetic_calibration_started_ms = millis();
  magnetic_calibration_previous_sample_ms =
      magnetic_calibration_started_ms - MAG_CALIBRATION_SAMPLE_INTERVAL_MS;
  magnetic_calibration_previous_report_ms = magnetic_calibration_started_ms;
  magnetic_calibration_sample_count = 0;
  magnetic_calibration_min_x_ut = magnetic_calibration_min_y_ut = 1.0e9f;
  magnetic_calibration_max_x_ut = magnetic_calibration_max_y_ut = -1.0e9f;
  send("INFO MAGCAL START ROTATE_TURNTABLE_360_DEG FOR_30SEC");
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
  send(message);
}

void process_magnetic_calibration(unsigned long now_ms) {
  if (!magnetic_calibration_active ||
      now_ms - magnetic_calibration_previous_sample_ms <
          MAG_CALIBRATION_SAMPLE_INTERVAL_MS) return;
  magnetic_calibration_previous_sample_ms = now_ms;

  float magnetic_x_ut, magnetic_y_ut, magnetic_z_ut;
  if (!imu->get_magnetometer(&magnetic_x_ut, &magnetic_y_ut, &magnetic_z_ut)) {
    magnetic_calibration_active = false;
    send("ERR MAGCAL SENSOR_READ_FAILED");
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
    const unsigned long capped_elapsed_ms = min(elapsed_ms, MAG_CALIBRATION_DURATION_MS);
    const unsigned long remaining_seconds =
        (MAG_CALIBRATION_DURATION_MS - capped_elapsed_ms + 999) / 1000;
    send("INFO MAGCAL ROTATING REMAIN=" + String(remaining_seconds) + "SEC");
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
    send("ERR MAGCAL INSUFFICIENT_ROTATION SPAN_X=" +
         String(range_x * 2.0f, 2) + " SPAN_Y=" + String(range_y * 2.0f, 2));
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
  attitude_estimator->reset(now_ms);
  send("OK MAGCAL RAM_ONLY OFFSET_X=" + String(magnetic_offset_x_ut, 3) +
       " OFFSET_Y=" + String(magnetic_offset_y_ut, 3) +
       " SCALE_X=" + String(magnetic_scale_x, 4) +
       " SCALE_Y=" + String(magnetic_scale_y, 4));
}

float apply_gyro_bias_correction(float gyro_z_deg_per_sec) {
  return gyro_bias_correction_enabled
      ? gyro_z_deg_per_sec - gyro_bias_z_deg_per_sec
      : gyro_z_deg_per_sec;
}

void apply_magnetic_calibration(float &magnetic_x_ut, float &magnetic_y_ut) {
  if (!magnetic_calibration_enabled) return;
  magnetic_x_ut = (magnetic_x_ut - magnetic_offset_x_ut) * magnetic_scale_x;
  magnetic_y_ut = (magnetic_y_ut - magnetic_offset_y_ut) * magnetic_scale_y;
}

void send_command_error() {
  send("ERR COMMANDS: est_j, est_m, est_f, biascal, biassave, magcal, magcal?, t, r<rpm>, s");
}

void receive_radio_commands(CommandHandler command_handler) {
  String usb;
  while (Serial.available()) usb += static_cast<char>(Serial.read());
  append(usb, command_handler);
  append(radio->get_text(), command_handler);
  if (buffer.length() && millis() - last_character_ms >= COMMAND_TIMEOUT_MS) {
    command_handler(buffer);
    buffer = "";
  }
}

void update_attitude_estimation(unsigned long now_ms) {
  attitude_estimator->update(*imu, now_ms);
}

void process_telemetry(unsigned long now_ms, unsigned long interval_ms) {
  if (telemetry && now_ms - last_telemetry_ms >= interval_ms) {
    last_telemetry_ms = now_ms;
    output_telemetry();
  }
}
