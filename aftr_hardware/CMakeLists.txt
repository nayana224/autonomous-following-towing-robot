cmake_minimum_required(VERSION 3.8)
project(aftr_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

# === User Code Begin === # 

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED) 

# 노드가 아닌 공유 라이브러리(.so)로 빌드합니다.
add_library(mdbot_hardware SHARED
  src/md_driver.cpp
  src/md_packet.cpp
  src/md_hardware_interface.cpp
)

target_include_directories(mdbot_hardware PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mdbot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

# 플러그인 XML 파일을 Export 합니다.
pluginlib_export_plugin_description_file(hardware_interface mdbot_hardware.xml)

install(TARGETS mdbot_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

ament_export_include_directories(include)
ament_export_libraries(mdbot_hardware)

# === User Code End === #

ament_package()
