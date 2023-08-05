#cmake version
cmake_minimum_required(VERSION 3.13.0)
#project name
project(alkaid_localization)
#c++ config
set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

#path setting
set(alkaid_localization_path ${CMAKE_CURRENT_SOURCE_DIR})
#roslib config
message("alkaid_localization$ 配置Ros库")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
roslib
)
include_directories( ${catkin_INCLUDE_DIRS})
catkin_package()


set(thirdparty_LIBRARIES "")
include(cmake/thirdparty_lib.cmake)

file(GLOB_RECURSE ALL_SRCS "module/*.cpp" "pipe/*.cpp" "pub/*.cpp" "sub/*.cpp"  "tools/*.cpp"  "data/*.cpp")
file(GLOB_RECURSE NODE_SRCS "node/*_node.cpp"   )
file(GLOB_RECURSE LIB_SRCS "thirdparty/*"   )
list(REMOVE_ITEM  ALL_SRCS  ${NODE_SRCS}   ${LIB_SRCS})


add_executable(module_test_node node/module_test_node.cpp ${ALL_SRCS})
target_link_libraries(module_test_node ${catkin_LIBRARIES}   ${thirdparty_LIBRARIES})
add_test(module_test_node COMMAND module_test_node)