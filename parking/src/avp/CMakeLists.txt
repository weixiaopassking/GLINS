cmake_minimum_required(VERSION 3.0.2)
project(avp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(engine_node ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/engine/engine_node.cpp)
target_link_libraries( engine_node  ${catkin_LIBRARIES})






