#ifndef RADIO_COMMANDS_H
#define RADIO_COMMANDS_H

#include <Arduino.h>

constexpr float RPM_TO_RAD_PER_SEC = TWO_PI / 60.0f;
constexpr unsigned long WHEEL_STARTUP_TIMEOUT_MS = 5000;
constexpr unsigned long WHEEL_RETRY_INTERVAL_MS = 250;

using CommandHandler = void (*)(String command);

void normalize_command(String &command);
void enable_angular_velocity_output();
void disable_angular_velocity_output();
bool ensure_wheel_is_connected();
void execute_start_command();
void execute_stop_command();
void execute_status_command();
void execute_set_target_angle_command(const String &angle_text);
void execute_set_kp_command(const String &kp_text);
void execute_set_kd_command(const String &kd_text);
void execute_gyro_bias_calibration_command();
void execute_gyro_bias_save_command();
void execute_magnetic_calibration_command();
void execute_magnetic_calibration_status_command();
void process_magnetic_calibration(unsigned long now_ms);
void send_command_error();

void receive_radio_commands(CommandHandler command_handler);
void send_message(const String &message);
void send_telemetry();
void process_telemetry(
    unsigned long now_ms, unsigned long telemetry_interval_ms);

#endif  // RADIO_COMMANDS_H

