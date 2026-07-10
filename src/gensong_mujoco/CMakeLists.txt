cmake_minimum_required(VERSION 3.16)
project(gensong_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS gensong_mujoco/gensong_mujoco_node.py
  gensong_mujoco/box_lift_policy.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
