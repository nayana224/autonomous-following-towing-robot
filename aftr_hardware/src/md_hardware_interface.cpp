// Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#include "mdbot_hardware/md_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mdbot_hardware
{


CallbackReturn MDHardwareInterface::on_init(const HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<StateInterface> MDHardwareInterface::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}


std::vector<CommandInterface> MDHardwareInterface::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}


CallbackReturn MDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MDHardwareInterface"), "Starting Hardware Activation...");
  
  std::string port = info_.hardware_parameters["port"];
  int baudrate = std::stoi(info_.hardware_parameters["baudrate"]);

  if (!driver_.init(port, baudrate)) {
    RCLCPP_ERROR(rclcpp::get_logger("MDHardwareInterface"), "Failed to initialize MDDriver!");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MDHardwareInterface"), "MD400T Hardware Activated Successfully.");
  return CallbackReturn::SUCCESS;
}


CallbackReturn MDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MDHardwareInterface"), "Deactivating Hardware...");
  
  driver_.release();

  return CallbackReturn::SUCCESS;
}


ReturnType MDHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Use one driver snapshot for both wheels in this read cycle.
  mdbot::RobotState current_state = driver_.readData(); 

  // MD400T quadrature encoder resolution: 16,384 ticks/rev.
  const double TICKS_PER_REV = 16384.0;
  const double RAD_PER_TICK = (2.0 * M_PI) / TICKS_PER_REV;
  const double RAD_S_PER_RPM = (2.0 * M_PI) / 60.0;

  hw_positions_[0] = static_cast<double>(current_state.pos_left) * RAD_PER_TICK;
  hw_positions_[1] = static_cast<double>(current_state.pos_right) * RAD_PER_TICK;

  hw_velocities_[0] = static_cast<double>(current_state.rpm_left) * RAD_S_PER_RPM;
  hw_velocities_[1] = static_cast<double>(current_state.rpm_right) * RAD_S_PER_RPM;

  return ReturnType::OK;
}


ReturnType MDHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Do not send motor commands while the initial command is NaN.
  if (std::isnan(hw_commands_[0]) || std::isnan(hw_commands_[1])) {
    return ReturnType::OK;
  }

  const double RAD_S_PER_RPM = (2.0 * M_PI) / 60.0;
  int32_t target_rpm_l = static_cast<int32_t>(hw_commands_[0] / RAD_S_PER_RPM);
  int32_t target_rpm_r = static_cast<int32_t>(hw_commands_[1] / RAD_S_PER_RPM);

  // The driver applies its current torque state to the velocity enable bits.
  driver_.setVelocity(target_rpm_l, target_rpm_r);

  return ReturnType::OK;
}

}  // namespace mdbot_hardware


PLUGINLIB_EXPORT_CLASS(
  mdbot_hardware::MDHardwareInterface,
  hardware_interface::SystemInterface)