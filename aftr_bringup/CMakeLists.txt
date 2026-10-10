cmake_minimum_required(VERSION 3.8)
project(aftr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# [User Code Begin]
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  scripts/99-mdbot-devices.rules
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(PROGRAMS
  scripts/create_udev_rules.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)
# [User Code End]

ament_package()
