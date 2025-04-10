cmake_minimum_required(VERSION 3.5)
project(asurt_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodeStatus.msg"
  "msg/Landmark.msg"
  "msg/LandmarkArray.msg"
  "msg/Roadstate.msg"
  DEPENDENCIES std_msgs
)

ament_package()
