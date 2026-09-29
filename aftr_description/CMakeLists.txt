cmake_minimum_required(VERSION 3.8)
project(aftr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# === User Code Begin === #

install(DIRECTORY
  urdf
  meshes
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

# === User Code End === #


ament_package()
