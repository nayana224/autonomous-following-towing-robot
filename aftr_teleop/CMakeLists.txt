cmake_minimum_required(VERSION 3.8)
project(aftr_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# [User Code Begin]
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mdbot_teleop_keyboard src/mdbot_teleop_keyboard.cpp)
ament_target_dependencies(
  mdbot_teleop_keyboard
  geometry_msgs
  rclcpp
)

install(TARGETS
  mdbot_teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)
# [User Code End]

ament_package()
