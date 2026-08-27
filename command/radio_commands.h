#ifndef RADIO_COMMANDS_H
#define RADIO_COMMANDS_H

#include <Arduino.h>

class HeptaCom;
class Bno055;
class UnitRollerI2C;
class HeptaEps;

using CommandHandler = void (*)(String command);

void begin_radio_commands(HeptaCom &com, Bno055 &sensor,
                          UnitRollerI2C &wheel, HeptaEps &eps);
void normalize_command(String &command);
void execute_set_wheel_speed_command(const String &speed_text);
void execute_wheel_start_command();
void enable_telemetry();
void execute_wheel_stop_command();
void disable_telemetry();
void send_command_error();
void receive_radio_commands(CommandHandler command_handler);
void process_telemetry(unsigned long now_ms);

#endif
