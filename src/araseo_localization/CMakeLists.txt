cmake_minimum_required(VERSION 3.8)
project(araseo_localization)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/pose_to_odom_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_to_odom
)

install(PROGRAMS
  scripts/wheel_odom_node.py
  scripts/auto_calibration_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
