#定义需求版本和工程名称
cmake_minimum_required(VERSION 3.0.2)
project(avp)

#cmake 编译配置
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

set(node_path   ${CMAKE_CURRENT_SOURCE_DIR}/node)


#第三方库配置
set(thirdpart_lib  "")
include(cmake/glog.cmake)
include(cmake/yamlcpp.cmake)

#ROS库配置
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

#自定义库配置
file(GLOB_RECURSE all_src  "*.cpp")
file(GLOB_RECURSE node_src "*_node.cpp")
list(REMOVE_ITEM  all_src  ${node_src})


add_executable(loclization_node   ${node_path}/loclization_node.cpp)
target_link_libraries( loclization_node    ${catkin_LIBRARIES} ${all_src})

add_executable(perception_node   ${node_path}/perception_node.cpp)
target_link_libraries( perception_node    ${catkin_LIBRARIES} ${all_src})




