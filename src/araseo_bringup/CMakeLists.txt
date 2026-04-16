cmake_minimum_required(VERSION 3.8)
project(araseo_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dynamixel_driver_node.py
  scripts/waypoint_follower.py
  scripts/map_viewer.py
  scripts/start_robot.sh
  scripts/stop_robot.sh
  scripts/install_service.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  systemd/araseo-robot.service
  DESTINATION share/${PROJECT_NAME}/systemd
)

ament_package()
