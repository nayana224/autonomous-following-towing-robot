# aftr_hardware Analysis

## 1. Overview
This package provides a ROS 2 Control based hardware interface for the MD400T motor driver.

Main structure:
- MDPacket: packet handling
- MDDriver: driver control
- MDHardwareInterface: ROS integration

## 2. Architecture
ros2_control → HardwareInterface → Driver → Packet → Serial

## 3. Data Flow
Command:
cmd_vel → write() → RPM conversion → setVelocity() → serial

Feedback:
serial → readData() → read() → rad conversion

## 4. Key Issues
- possible missing includes
- no joint-count validation
- no parameter validation
- no explicit release behavior during deactivate
- possible RPM overflow

## 5. Reading Order
1. md_defines.hpp
2. md_packet.cpp
3. md_driver.cpp
4. md_hardware_interface.cpp
