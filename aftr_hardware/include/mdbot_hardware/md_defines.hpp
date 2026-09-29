// Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#pragma once

#include <cstdint>

namespace mdbot
{

  constexpr uint8_t HEADER_RMID = 183; // MD400T receiver address.
  constexpr uint8_t HEADER_TMID = 184; // PC sender address.
  constexpr uint8_t DRIVER_ID = 1; // MD400T driver ID.

  constexpr uint8_t LEN_DATA_1 = 1; // Reset, Torque Off
  constexpr uint8_t LEN_DATA_3 = 3; // Data Request
  constexpr uint8_t LEN_DATA_7 = 7; // Velocity Cmd
  constexpr uint8_t LEN_DATA_17 = 17;  // Feedback Data

  // PID: Parameter ID
  enum class PID : uint8_t 
  {
    DATA_REQ = 4,
    COMMAND = 10,
    USE_EMER_WS = 76,
    MOTOR1_MAIN_DATA = 193,
    MOTOR2_MAIN_DATA = 200,
    TORQUE_OFF = 5,
    BRAKE = 6,
    VEL_CMD = 207
  };

  enum class CMD_NUM : uint8_t
  {
    ALARM_RESET = 8,
    CMD_POSI_RESET = 10
  };

  struct RobotState
  {
    int32_t rpm_left;
    int32_t rpm_right;
    int32_t pos_left;
    int32_t pos_right;
  };

} // namespace mdbot
