cmake_minimum_required(VERSION 3.8)
project(dynamic_nav_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds config maps models textures
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
