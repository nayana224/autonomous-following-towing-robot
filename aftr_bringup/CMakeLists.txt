cmake_minimum_required(VERSION 3.8)
project(aftr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  scripts
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Preserve executable permissions for the installed udev setup script.
install(PROGRAMS
  scripts/create_udev_rules.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
