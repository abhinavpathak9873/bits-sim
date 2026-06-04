cmake_minimum_required(VERSION 3.8)
project(dynamic_nav_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

add_library(dynamic_nav_scripted_actor SHARED src/scripted_actor_plugin.cpp)
target_include_directories(dynamic_nav_scripted_actor PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(dynamic_nav_scripted_actor ${GAZEBO_LIBRARIES})

install(TARGETS dynamic_nav_scripted_actor
  LIBRARY DESTINATION lib
)

ament_package()
