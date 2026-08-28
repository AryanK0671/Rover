cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    meshes
    config
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/rover_teleop_key.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
