#include "radio_commands.h"

#include <stdlib.h>

#include "../../com/hepta_com.h"
#include "../../drv/imu9axis_bno055.h"
#include "../../drv/unit_rolleri2c.hpp"
#include "../../eps/hepta_eps.h"

namespace {
constexpr uint32_t USB_BAUD_RATE = 115200;
constexpr uint32_t XBEE_BAUD_RATE = 38400;
constexpr uint8_t WHEEL_SDA_PIN = 6;
constexpr uint8_t WHEEL_SCL_PIN = 7;
constexpr int32_t MAX_WHEEL_SPEED_RPM = 3000;
constexpr int32_t WHEEL_MAX_CURRENT = 100000;
constexpr float WHEEL_CURRENT_REGISTER_PER_MA = 100.0f;
constexpr unsigned long TELEMETRY_INTERVAL_MS = 1000;
constexpr unsigned long COMMAND_TIMEOUT_MS = 100;
constexpr size_t COMMAND_BUFFER_SIZE = 32;

HeptaCom *radio = nullptr;
Bno055 *gyro = nullptr;
UnitRollerI2C *reaction_wheel = nullptr;
HeptaEps *satellite_eps = nullptr;
String command_buffer;
int32_t target_speed_rpm = 0;
bool target_speed_is_set = false;
bool wheel_connected = false;
bool wheel_is_running = false;
bool telemetry_enabled = false;
unsigned long last_command_character_ms = 0;
unsigned long last_telemetry_ms = 0;

void send_message(const String &message) {
  radio->send_text(message + "\r\n");
  Serial.println(message);
}

bool parse_speed(const String &text, int32_t &speed_rpm) {
  String value_text = text;
  value_text.trim();
  if (value_text.length() == 0) return false;
  char *end = nullptr;
  const long value = strtol(value_text.c_str(), &end, 10);
  if (end == value_text.c_str() || *end != '\0' ||
      value < -MAX_WHEEL_SPEED_RPM || value > MAX_WHEEL_SPEED_RPM) {
    return false;
  }
  speed_rpm = static_cast<int32_t>(value);
  return true;
}

bool ensure_wheel_connected() {
  if (wheel_connected) return true;
  wheel_connected = reaction_wheel->connect(5000, 250);
  if (!wheel_connected) send_message("ERR WHEEL_DISCONNECTED");
  return wheel_connected;
}

void append_command_text(const String &text, CommandHandler command_handler) {
  for (unsigned int i = 0; i < text.length(); ++i) {
    const char character = text.charAt(i);
    if (character == '\r' || character == '\n') {
      if (command_buffer.length() > 0) command_handler(command_buffer);
      command_buffer = "";
    } else if (command_buffer.length() < COMMAND_BUFFER_SIZE - 1) {
      command_buffer += character;
      last_command_character_ms = millis();
    } else {
      command_buffer = "";
      send_message("ERR COMMAND_TOO_LONG");
    }
  }
}

void receive_commands(CommandHandler command_handler) {
  append_command_text(radio->get_text(), command_handler);
  String usb_text;
  while (Serial.available() > 0) usb_text += static_cast<char>(Serial.read());
  append_command_text(usb_text, command_handler);
  if (command_buffer.length() > 0 &&
      millis() - last_command_character_ms >= COMMAND_TIMEOUT_MS) {
    command_handler(command_buffer);
    command_buffer = "";
  }
}

void send_telemetry() {
  float gyro_x_dps = 0.0f;
  float gyro_y_dps = 0.0f;
  float gyro_z_dps = 0.0f;
  gyro->sen_gyro(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);
  const int32_t measured_speed_rpm =
      wheel_connected ? reaction_wheel->getSpeedReadbackRpm() : 0;
  const float wheel_current_ma =
      wheel_connected
          ? reaction_wheel->getCurrentReadback() /
                WHEEL_CURRENT_REGISTER_PER_MA
          : 0.0f;
  const float battery_voltage_v = satellite_eps->get_battery_voltage();

  String line;
  line.reserve(140);
  line += "WheelSpeed_RPM:" + String(measured_speed_rpm);
  line += "\tWheelCurrent_mA:" + String(wheel_current_ma, 2);
  line += "\tSatelliteAngularRate_dps:" + String(gyro_z_dps, 2);
  line += "\tBatteryVoltage_V:" + String(battery_voltage_v, 3);
  Serial.println(line);
  radio->send_text(line + "\r\n");
}
}

void begin_radio_commands(HeptaCom &com, Bno055 &sensor,
                          UnitRollerI2C &wheel, HeptaEps &eps) {
  radio = &com;
  gyro = &sensor;
  reaction_wheel = &wheel;
  satellite_eps = &eps;
  Serial.begin(USB_BAUD_RATE);
  radio->begin(XBEE_BAUD_RATE);
  gyro->begin();
  satellite_eps->init();
  delay(500);
  wheel_connected = reaction_wheel->begin(
      &Wire1, WHEEL_SDA_PIN, WHEEL_SCL_PIN, I2C_ADDR, 100000, 5000, 250);
  send_message(wheel_connected ? "I2C WHEEL TRUE" : "I2C WHEEL FALSE");
  send_message(
      "READY: r<rpm>=SET, a=START, t=TELEMETRY, s=STOP, "
      "ts=TELEMETRY_STOP");
}

void normalize_command(String &command) {
  command.trim();
  command.toLowerCase();
}

void execute_set_wheel_speed_command(const String &speed_text) {
  int32_t requested_speed_rpm;
  if (!parse_speed(speed_text, requested_speed_rpm)) {
    send_message("ERR USAGE r<rpm> (-3000..3000)");
    return;
  }
  target_speed_rpm = requested_speed_rpm;
  target_speed_is_set = true;
  if (wheel_is_running) {
    if (!ensure_wheel_connected()) return;
    reaction_wheel->setSpeedRpm(target_speed_rpm);
    send_message("OK WHEEL RPM APPLIED " + String(target_speed_rpm));
  } else {
    send_message("OK WHEEL RPM SET " + String(target_speed_rpm));
  }
}

void execute_wheel_start_command() {
  if (!target_speed_is_set) {
    send_message("ERR SET RPM FIRST: r<rpm>");
    return;
  }
  if (!ensure_wheel_connected()) return;
  reaction_wheel->start(target_speed_rpm, WHEEL_MAX_CURRENT);
  wheel_is_running = true;
  send_message("OK WHEEL START " + String(target_speed_rpm) + " RPM");
}

void enable_telemetry() {
  telemetry_enabled = true;
  last_telemetry_ms = millis() - TELEMETRY_INTERVAL_MS;
  send_message("OK TELEMETRY ON");
}

void execute_wheel_stop_command() {
  if (!ensure_wheel_connected()) return;
  reaction_wheel->stop();
  wheel_is_running = false;
  send_message("OK WHEEL STOP");
}

void disable_telemetry() {
  telemetry_enabled = false;
  send_message("OK TELEMETRY OFF");
}

void send_command_error() {
  send_message("ERR COMMANDS: r<rpm>, a, t, s, ts");
}

void receive_radio_commands(CommandHandler command_handler) {
  if (radio != nullptr) receive_commands(command_handler);
}

void process_telemetry(unsigned long now_ms) {
  if (telemetry_enabled &&
      now_ms - last_telemetry_ms >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_ms = now_ms;
    send_telemetry();
  }
}
