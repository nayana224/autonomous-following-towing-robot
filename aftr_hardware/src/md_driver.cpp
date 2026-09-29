// Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#include "mdbot_hardware/md_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>

namespace mdbot
{

MDDriver::MDDriver()
: is_torque_on_(true)
{
  current_state_.rpm_left = 0;
  current_state_.rpm_right = 0;
  current_state_.pos_left = 0;
  current_state_.pos_right = 0;
}

MDDriver::~MDDriver()
{
  release();
}

bool MDDriver::init(std::string port, int baud)
{
  try {
    serial_.setPort(port);
    serial_.setBaudrate(baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(50);
    serial_.setTimeout(to);
    serial_.open();
  } catch (serial::IOException& e) {
    return false;
  }

  if (!serial_.isOpen()) {
    return false;
  }
  // Configure the CTRL emergency input before the startup command sequence.
  setUseEmergencySwitch(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Let the controller settle after resetting its alarm.
  resetAlarm();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Clear any prior motion command before reading motor state.
  setVelocity(0, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Reset odometry to the boot reference.
  resetEncoder();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  readData();
  return true;
}



void MDDriver::release()
{
  if (serial_.isOpen())
  {
    try 
    {
      // Command zero RPM before disabling torque.
      setVelocity(0, 0); 
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Let zero RPM reach the controller.

      setTorque(false); 
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Let torque-off transmit before closing.

      serial_.close();
      
      std::cout << "[MDDriver] Released: Motor Stopped & Torque OFF." << std::endl;
    } catch (...) {
      // Shutdown can fail after disconnection; torque-off may then be unconfirmed.
      std::cerr << "[MDDriver] Critical: Could not send Torque OFF command (Hardware Disconnected)." << std::endl;
    }
    serial_.close();
  }
}


RobotState MDDriver::readData()
{
  if (!serial_.isOpen()) return current_state_;

  size_t available_len = serial_.available();
  if (available_len > 0)
  {
    std::vector<uint8_t> temp_buffer;
    serial_.read(temp_buffer, available_len);
    rx_buffer_.insert(rx_buffer_.end(), temp_buffer.begin(), temp_buffer.end());
  }

  std::vector<std::vector<uint8_t>> packets = packet_handler_.parseBuffer(rx_buffer_);

  for (const auto& packet : packets) 
  {
    uint8_t pid_val = packet_handler_.getPID(packet);
    std::vector<uint8_t> data = packet_handler_.getData(packet);

    // PID 210 carries dual-channel motor state.
    // RPM and encoder fields follow the driver's fixed packet layout.
    if (pid_val == 210 && data.size() >= 18) 
    {
      // Decode motor fields as little-endian values.
      int16_t rpm1 = data[0] | (data[1] << 8);
      int32_t pos1 = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);

      int16_t rpm2 = data[9] | (data[10] << 8);
      int32_t pos2 = data[14] | (data[15] << 8) | (data[16] << 16) | (data[17] << 24); 
      

      current_state_.rpm_left = rpm1;
      current_state_.pos_left = pos1;
      current_state_.rpm_right = rpm2;
      current_state_.pos_right = pos2;
    }
  }
  return current_state_;
}


void MDDriver::sendPacket(PID pid, const std::vector<uint8_t>& data)
{
  if (!serial_.isOpen()) return;

  std::vector<uint8_t> packet = packet_handler_.createPacket(pid, data);

  try
  {
    serial_.write(packet);
  } 
  catch(...) 
  {
    std::cerr << "[MDDriver] Unknown Error during packet send.\n";
    return;
  }
}


void MDDriver::setVelocity(int32_t rpm_left, int32_t rpm_right)
{
  // PID207: [Enable1][RPM1_L][RPM1_H][Enable2][RPM2_L][RPM2_H][ReqPID]
  std::vector<uint8_t> data;
  uint8_t enable = is_torque_on_ ? 1 : 0; // Do not set the velocity-enable bit while torque is off.

  data.emplace_back(enable);
  data.emplace_back(rpm_left & 0xFF);
  data.emplace_back((rpm_left >> 8) & 0xFF);

  data.emplace_back(enable);
  data.emplace_back(rpm_right & 0xFF);
  data.emplace_back((rpm_right >> 8) & 0xFF);

  // Return Value Request: PID210(MAIN_DATA)
  data.emplace_back(2);

  sendPacket(PID::VEL_CMD, data);
}


void MDDriver::setTorque(bool on)
{
  is_torque_on_ = on;
  if (on) 
  {
    // Reset the alarm when torque is enabled.
    resetAlarm();
  }
  else 
  {
    // Use PID 174 to disable torque.
    std::vector<uint8_t> data = {1};
    sendPacket(PID::TORQUE_OFF, data);
  }
}

void MDDriver::setBrake(bool on)
{
  // PID 175 -> Data 1(ON) / 0(OFF)
  std::vector<uint8_t> data;
  data.emplace_back(on ? 1 : 0);
  sendPacket(PID::BRAKE, data);
}

void MDDriver::setUseEmergencySwitch(bool use)
{
  // PID 76 configures the CTRL emergency-switch input.
  std::vector<uint8_t> data;
  data.emplace_back(use ? 1 : 0);
  sendPacket(PID::USE_EMER_WS, data);
}


void MDDriver::resetAlarm()
{
  // PID 10 -> Data 8
  std::vector<uint8_t> data = {8};
  sendPacket(PID::COMMAND, data);
}

void MDDriver::resetEncoder()
{
  // PID 10 -> Data 10
  std::vector<uint8_t> data = {10};
  sendPacket(PID::COMMAND, data);
}


} // namespace mdbot
