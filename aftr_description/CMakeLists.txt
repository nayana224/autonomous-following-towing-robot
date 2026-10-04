cmake_minimum_required(VERSION 3.8)
project(aftr_description)

find_package(ament_cmake REQUIRED)

# [User Code Begin]
install(DIRECTORY
  urdf
  meshes
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)
# [User Code End]

ament_package()
