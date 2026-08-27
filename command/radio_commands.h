#ifndef RADIO_COMMANDS_H
#define RADIO_COMMANDS_H

#include <Arduino.h>
#include "../adcs/angular_estimation.h"

class HeptaCom;
class HeptaEps;
class HeptaSensor;
class UnitRollerI2C;
using CommandHandler = void (*)(String command);

void begin_adcs_module(HeptaCom &com, HeptaEps &eps, HeptaSensor &sensor,
                       UnitRollerI2C &wheel, AngularEstimation &estimation);
void normalize_command(String &command);
void execute_estimation_mode_command(AngularEstimation::Mode mode);
void enable_telemetry();
void execute_set_wheel_speed_command(const String &speed_text);
void execute_wheel_stop_command();
void execute_gyro_bias_calibration_command();
void execute_gyro_bias_save_command();
void execute_magnetic_calibration_command();
void execute_magnetic_calibration_status_command();
void process_magnetic_calibration(unsigned long now_ms);
float apply_gyro_bias_correction(float gyro_z_deg_per_sec);
void apply_magnetic_calibration(float &magnetic_x_ut, float &magnetic_y_ut);
void send_command_error();
void receive_radio_commands(CommandHandler command_handler);
void update_attitude_estimation(unsigned long now_ms);
void process_telemetry(unsigned long now_ms, unsigned long interval_ms);

#endif
