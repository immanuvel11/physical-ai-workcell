cmake_minimum_required(VERSION 3.8)
project(pai_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DetectedObject.msg msg/Relation.msg msg/SceneGraph.msg msg/SafetyStatus.msg msg/TaskStatus.msg
  action/Pick.action action/Place.action action/MoveTo.action action/RunInstruction.action
  DEPENDENCIES std_msgs geometry_msgs)
ament_package()
