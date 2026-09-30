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
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/rover_teleop_key.py
    scripts/scan_to_points.py
    scripts/cmd_vel_to_rover.py
    scripts/spawn_obstacle.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rover_description test/test_rover_description.py)
  ament_add_pytest_test(test_obstacle_and_points test/test_obstacle_and_points.py)
endif()

ament_package()
