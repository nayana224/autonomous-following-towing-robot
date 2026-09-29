// Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#pragma once

#include <string>
#include <vector>
#include <serial/serial.h>

#include "mdbot_hardware/md_defines.hpp"
#include "mdbot_hardware/md_packet.hpp"

namespace mdbot
{

/// Serial driver for MD motor commands and feedback.
class MDDriver
{
public:
  MDDriver();
  ~MDDriver();

  /// Open the motor serial port and initialize the driver.
  bool init(std::string port, int baud);

  /// Stop the motor and close the serial connection.
  void release();

  /// Read and return the latest motor state.
  RobotState readData();

  
  RobotState getRobotState() const { return current_state_; }
  int32_t getLeftEncoder() const { return current_state_.pos_left; }
  int32_t getRightEncoder() const { return current_state_.pos_right; }
  int32_t getLeftRPM() const { return current_state_.rpm_left; }
  int32_t getRightRPM() const { return current_state_.rpm_right; } 

  void setVelocity(int32_t rpm_left, int32_t rpm_right);
  void setTorque(bool on);
  void setBrake(bool on);
  void resetAlarm();
  void resetEncoder();
  void setUseEmergencySwitch(bool use);


private:
  serial::Serial serial_;
  MDPacket packet_handler_; 

  std::vector<uint8_t> rx_buffer_;
  RobotState current_state_;

  bool is_torque_on_;

  void sendPacket(PID pid, const std::vector<uint8_t>& data = {});
  


};


} // namespace mdbot

