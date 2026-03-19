cmake_minimum_required(VERSION 3.8)
project(ros_hardware)

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
find_package(robot_msgs REQUIRED)


add_library(ros_hardware SHARED
  src/ros_hardware_interface.cpp
)

target_include_directories(ros_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ros_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  robot_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface ros_hardware.xml)


install(TARGETS ros_hardware
  EXPORT export_ros_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES ros_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ros_hardware)
ament_export_targets(export_ros_hardware)

ament_package()