cmake_minimum_required(VERSION 3.8)
project(micro_ros_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_messages REQUIRED)


add_library(micro_ros_hardware SHARED
  src/micro_ros_hardware_interface.cpp
)

target_include_directories(micro_ros_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(micro_ros_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  robot_messages
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface micro_ros_hardware.xml)


install(TARGETS micro_ros_hardware
  EXPORT export_micro_ros_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES micro_ros_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(micro_ros_hardware)
ament_export_targets(export_micro_ros_hardware)

ament_package()