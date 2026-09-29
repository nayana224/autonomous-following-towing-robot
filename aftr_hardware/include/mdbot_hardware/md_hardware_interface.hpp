// Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#pragma once

#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mdbot_hardware/md_driver.hpp"


namespace mdbot_hardware
{
using CallbackReturn = hardware_interface::CallbackReturn;
using ReturnType = hardware_interface::return_type;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;
using SystemInterface = hardware_interface::SystemInterface;
using HardwareInfo = hardware_interface::HardwareInfo;


/// Expose MD motor state and commands through ros2_control.
class MDHardwareInterface 
: public SystemInterface
{
public:
  /// Define shared-pointer aliases required by ROS 2.
  RCLCPP_SHARED_PTR_DEFINITIONS(MDHardwareInterface)

  /// Initialize ros2_control from URDF hardware information.
  CallbackReturn on_init(const HardwareInfo& info) override;

  /// Expose wheel position and velocity states to ros2_control.
  std::vector<StateInterface> export_state_interfaces() override;

  /// Expose wheel velocity commands to ros2_control.
  std::vector<CommandInterface> export_command_interfaces() override;

  /// Open and initialize the motor connection.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /// Stop the motor and release the connection.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /// Convert encoder ticks and RPM to ROS wheel state units.
  ReturnType read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Convert wheel velocity commands to motor RPM.
  ReturnType write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  mdbot::MDDriver driver_;

  std::vector<double> hw_commands_; // [0]: Left Velocity, [1]: Right Velocity
  std::vector<double> hw_positions_; // [0]: Left Position, [1]: Right Position
  std::vector<double> hw_velocities_; // [0]: Left Velocity, [1]: Right Velocity
};

}  // namespace mdbot_hardware

