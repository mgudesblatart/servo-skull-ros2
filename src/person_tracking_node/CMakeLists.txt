cmake_minimum_required(VERSION 3.8)
project(person_tracking_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(servo_skull_msgs REQUIRED)
find_package(person_sensor_node REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(person_tracking_node_exe src/person_tracking_node.cpp)
ament_target_dependencies(person_tracking_node_exe rclcpp servo_skull_msgs person_sensor_node)


ament_export_dependencies(rosidl_default_runtime)

install(TARGETS person_tracking_node_exe
  DESTINATION lib/${PROJECT_NAME})

ament_package()
