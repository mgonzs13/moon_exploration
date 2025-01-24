cmake_minimum_required(VERSION 3.5)
project(moon_exploration_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch grammars
  DESTINATION share/${PROJECT_NAME}
)

ament_package()