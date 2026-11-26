cmake_minimum_required(VERSION 3.8)
project(vacuum_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Submode.msg"
  "msg/StuckEvent.msg"
  "msg/Notification.msg"
  "msg/OffloadStatus.msg"
  "action/Escape.action"
  "msg/Room.msg"
  "msg/RoomArray.msg"
  "srv/SetRoomLabel.srv"
  "action/CoverRoom.action"
  "action/Explore.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
