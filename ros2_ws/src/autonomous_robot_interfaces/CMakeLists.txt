cmake_minimum_required(VERSION 3.8)
project(autonomous_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "msg/ObstacleWarning.msg"
  "msg/SemanticObstacle.msg"
  "msg/SemanticObstacleArray.msg"
  "msg/SignDetection.msg"
  "msg/SignDetectionArray.msg"
  DEPENDENCIES std_msgs
)

ament_package()
