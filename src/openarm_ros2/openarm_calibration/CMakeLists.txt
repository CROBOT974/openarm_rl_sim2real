cmake_minimum_required(VERSION 3.22)
project(openarm_calibration)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aruco_pose_node.py
  scripts/auto_sample.py
  scripts/publish_calib.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
