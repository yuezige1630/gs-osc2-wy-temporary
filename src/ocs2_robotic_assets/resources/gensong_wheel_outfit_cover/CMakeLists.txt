cmake_minimum_required(VERSION 3.16)
project(gensong_wheel_outfit_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes mjcf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
