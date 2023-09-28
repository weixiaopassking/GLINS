#cmake 
cmake_minimum_required(VERSION 3.13.0)
project(light_roadmap)
set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

#Ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)



catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)




#自定义
file( GLOB_RECURSE all_SRCS 
"module/*.cpp" 
"pipeline/*.cpp"
"pub/*.cpp"
"sub/*.cpp"
"msg/*.cpp"
"tools/*.cpp")

set(thirdparty_LIBRARIES "")
include(cmake/thirdparty_lib.cmake)
add_executable(node node/node.cpp ${all_SRCS})
target_link_libraries(node   ${catkin_LIBRARIES}  ${thirdparty_LIBRARIES})