#cmake version and project name 
cmake_minimum_required(VERSION 3.0.2)
project(glins)

# Build configuration
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)



# Ros libraries configuration
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()


# Third part libraries configuration
set( THIRD_PART_LIBRARIES "")
include(cmake/glog.cmake)
include(cmake/geographiclib.cmake)
include(cmake/eigen.cmake)
include(cmake/yamlcpp.cmake)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM  ALL_SRCS  ${NODE_SRCS})

# Link and Build
add_executable(preprocess_node src/app/preprocess_node.cpp  ${ALL_SRCS})
target_link_libraries(preprocess_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES})