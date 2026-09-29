cmake_minimum_required(VERSION 3.8)
project(autonomous_robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/decision_engine_node
  scripts/navigation_visualizer_node
  scripts/dynamic_obstacles_node
  scripts/dashboard_backend
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch maps web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
