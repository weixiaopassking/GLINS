#cmake version
cmake_minimum_required(VERSION 3.13.0)
#project name
project(alkaid_localization)
#c++ config
set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#roslib config
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)
include_directories( ${catkin_INCLUDE_DIRS})
catkin_package()
#thirdparty
set(thirdparty_LIBRARIES "")
include(cmake/thirdparty_lib.cmake)
#self-defining party
file(GLOB_RECURSE ALL_SRCS "module/*.cpp" "pipe/*.cpp" "pub/*.cpp" "sub/*.cpp"  "tools/*.cpp"  "data/*.cpp")

#exe
option(USE_TEST  "module test code" OFF)

if(USE_TEST)
message("now test module")
add_executable(module_test_node node/module_test_node.cpp ${ALL_SRCS})
target_link_libraries(module_test_node ${catkin_LIBRARIES}  ${thirdparty_LIBRARIES})
add_test(module_test_node COMMAND module_test_node)
else()
message("now test node")
add_executable(localization_node node/localization_node.cpp ${ALL_SRCS})
target_link_libraries(localization_node ${catkin_LIBRARIES}  ${thirdparty_LIBRARIES})
endif()





