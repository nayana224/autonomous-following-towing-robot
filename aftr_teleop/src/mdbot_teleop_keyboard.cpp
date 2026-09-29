// Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
//
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{

volatile sig_atomic_t g_request_shutdown = 0;

void signalHandler(int)
{
  g_request_shutdown = 1;
}

class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    if (tcgetattr(STDIN_FILENO, &original_settings_) != 0) {
      enabled_ = false;
      return;
    }

    termios raw_settings = original_settings_;
    raw_settings.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw_settings.c_cc[VMIN] = 0;
    raw_settings.c_cc[VTIME] = 1;

    enabled_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw_settings) == 0;
  }

  ~TerminalRawMode()
  {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_settings_);
    }
  }

  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode & operator=(const TerminalRawMode &) = delete;

  bool enabled() const
  {
    return enabled_;
  }

private:
  termios original_settings_{};
  bool enabled_{false};
};

}  // namespace

/// Publish bounded keyboard velocity commands and request a stop on exit.
class MDBotTeleopKeyboard : public rclcpp::Node
{
public:
  MDBotTeleopKeyboard()
  : Node("mdbot_teleop_keyboard")
  {
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    linear_step_ = declare_parameter<double>("linear_step", 0.05);
    angular_step_ = declare_parameter<double>("angular_step", 0.10);
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 0.50);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 1.00);
    linear_direction_ = declare_parameter<double>("linear_direction", 1.0);
    angular_direction_ = declare_parameter<double>("angular_direction", 1.0);
    publish_zero_on_exit_ = declare_parameter<bool>("publish_zero_on_exit", true);

    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    publish_timer_ = create_wall_timer(
      50ms,
      [this]() {
        publishVelocity(target_linear_velocity_, target_angular_velocity_);
      });

    printHelp();
  }

  ~MDBotTeleopKeyboard() override
  {
    // Request zero velocity so shutdown does not leave the last key command active.
    if (publish_zero_on_exit_) {
      publishVelocity(0.0, 0.0);
    }
  }

  void spinKeyboard()
  {
    TerminalRawMode terminal;
    if (!terminal.enabled()) {
      RCLCPP_ERROR(get_logger(), "failed to configure terminal raw mode");
      return;
    }

    rclcpp::WallRate rate(20.0);
    while (rclcpp::ok() && !g_request_shutdown) {
      rclcpp::spin_some(shared_from_this());

      const int key = readKey();
      if (key > 0) {
        handleKey(static_cast<char>(key));
      }

      rate.sleep();
    }
  }

private:
  int readKey()
  {
    unsigned char c = 0;
    const ssize_t result = read(STDIN_FILENO, &c, 1);
    if (result == 1) {
      return c;
    }
    return -1;
  }

  void handleKey(char key)
  {
    switch (key) {
      case 'w':
      case 'W':
        if (target_linear_velocity_ < 0.0) {
          target_linear_velocity_ = linear_step_;
        } else {
          target_linear_velocity_ += linear_step_;
        }
        break;
      case 'x':
      case 'X':
        if (target_linear_velocity_ > 0.0) {
          target_linear_velocity_ = -linear_step_;
        } else {
          target_linear_velocity_ -= linear_step_;
        }
        break;
      case 'a':
      case 'A':
        if (target_angular_velocity_ < 0.0) {
          target_angular_velocity_ = angular_step_;
        } else {
          target_angular_velocity_ += angular_step_;
        }
        break;
      case 'd':
      case 'D':
        if (target_angular_velocity_ > 0.0) {
          target_angular_velocity_ = -angular_step_;
        } else {
          target_angular_velocity_ -= angular_step_;
        }
        break;
      case 's':
      case 'S':
      case ' ':
        target_linear_velocity_ = 0.0;
        target_angular_velocity_ = 0.0;
        break;
      case 'q':
      case 'Q':
        target_linear_velocity_ = nextPositive(target_linear_velocity_, linear_step_);
        target_angular_velocity_ = nextPositive(target_angular_velocity_, angular_step_);
        break;
      case 'e':
      case 'E':
        target_linear_velocity_ = nextPositive(target_linear_velocity_, linear_step_);
        target_angular_velocity_ = nextNegative(target_angular_velocity_, angular_step_);
        break;
      case 'z':
      case 'Z':
        target_linear_velocity_ = nextNegative(target_linear_velocity_, linear_step_);
        target_angular_velocity_ = nextNegative(target_angular_velocity_, angular_step_);
        break;
      case 'c':
      case 'C':
        target_linear_velocity_ = nextNegative(target_linear_velocity_, linear_step_);
        target_angular_velocity_ = nextPositive(target_angular_velocity_, angular_step_);
        break;
      case '\x03':
        g_request_shutdown = 1;
        return;
      default:
        return;
    }

    target_linear_velocity_ = clamp(
      target_linear_velocity_, -max_linear_velocity_, max_linear_velocity_);
    target_angular_velocity_ = clamp(
      target_angular_velocity_, -max_angular_velocity_, max_angular_velocity_);

    publishVelocity(target_linear_velocity_, target_angular_velocity_);
    printVelocity();
  }

  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  static double nextPositive(double current_value, double step)
  {
    if (current_value < 0.0) {
      return step;
    }
    return current_value + step;
  }

  static double nextNegative(double current_value, double step)
  {
    if (current_value > 0.0) {
      return -step;
    }
    return current_value - step;
  }

  void publishVelocity(double linear_velocity, double angular_velocity)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_velocity * linear_direction_;
    twist.angular.z = angular_velocity * angular_direction_;
    cmd_publisher_->publish(twist);
  }

  void printHelp()
  {
    std::cout << "\nMDBOT keyboard teleoperation\n"
              << "---------------------------\n"
              << "Moving around:\n"
              << "   q    w    e\n"
              << "   a    s    d\n"
              << "   z    x    c\n\n"
              << "w/x : increase/decrease linear velocity\n"
              << "a/d : increase/decrease angular velocity\n"
              << "s or space : stop\n"
              << "CTRL-C : quit\n\n"
              << "CAUTION: run only when no other node is publishing /cmd_vel.\n"
              << "cmd_topic: " << cmd_topic_ << "\n"
              << "max_linear_velocity: " << max_linear_velocity_ << " m/s\n"
              << "max_angular_velocity: " << max_angular_velocity_ << " rad/s\n"
              << "linear_direction: " << linear_direction_ << "\n"
              << "angular_direction: " << angular_direction_ << "\n\n";
  }

  void printVelocity()
  {
    std::cout << "linear=" << target_linear_velocity_
              << " m/s, angular=" << target_angular_velocity_
              << " rad/s"
              << " -> published linear=" << target_linear_velocity_ * linear_direction_
              << ", angular=" << target_angular_velocity_ * angular_direction_
              << std::endl;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::string cmd_topic_;
  double linear_step_{0.05};
  double angular_step_{0.10};
  double max_linear_velocity_{0.50};
  double max_angular_velocity_{1.00};
  double linear_direction_{1.0};
  double angular_direction_{1.0};
  double target_linear_velocity_{0.0};
  double target_angular_velocity_{0.0};
  bool publish_zero_on_exit_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signalHandler);

  auto node = std::make_shared<MDBotTeleopKeyboard>();
  node->spinKeyboard();

  rclcpp::shutdown();
  return 0;
}
