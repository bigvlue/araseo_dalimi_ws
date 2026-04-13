cmake_minimum_required(VERSION 3.8)
project(araseo_dalimi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/LaneInfo.msg"
  "msg/TrafficSign.msg"
  "msg/MissionStatus.msg"
  "msg/FleetStatus.msg"
  "msg/PinkyGps.msg"
  "srv/SetMission.srv"
  "srv/SetDrivingMode.srv"
  "srv/GetMissionStatus.srv"
  "action/NavigateToGoal.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
