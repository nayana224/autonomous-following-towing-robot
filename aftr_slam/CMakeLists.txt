cmake_minimum_required(VERSION 3.8)
project(aftr_slam)

find_package(ament_cmake REQUIRED)

# [User Code Begin]
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
# [User Code End]

ament_package()
