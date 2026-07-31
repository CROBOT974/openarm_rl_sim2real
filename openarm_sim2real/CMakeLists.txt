cmake_minimum_required(VERSION 3.8)
project(openarm_sim2real)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/rl_policy_node.py
  scripts/rl_policy_node_cycle.py
  scripts/send_forward_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
