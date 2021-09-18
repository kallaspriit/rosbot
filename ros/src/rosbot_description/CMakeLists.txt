cmake_minimum_required(VERSION 3.5)
project(rosbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
