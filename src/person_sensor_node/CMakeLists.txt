cmake_minimum_required(VERSION 3.22)
project(person_sensor_node)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonDetection.msg"
)

ament_package()
