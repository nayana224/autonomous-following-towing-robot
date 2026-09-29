cmake_minimum_required(VERSION 3.8)
project(aftr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# === User Code Begin === #

install(DIRECTORY
  launch
  config
  scripts
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# 쉘 스크립트에 실행 권한(Executable)을 부여하여 설치하는 옵션 추가
install(PROGRAMS
  scripts/create_udev_rules.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

# === User Code End === #
ament_package()
