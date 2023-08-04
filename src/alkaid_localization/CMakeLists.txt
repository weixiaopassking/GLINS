#cmake 版本
cmake_minimum_required(VERSION 3.13.0)
#工程名
project(alkaid_localization)
#c++配置
set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

#路径设置
set(alkaid_localization_path  ${CMAKE_CURRENT_SOURCE_DIR})


#ros库配置
message("alkaid_localization$ 配置Ros库")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
roslib
)
include_directories( ${catkin_INCLUDE_DIRS})
catkin_package()



#第三方库配置
set(thirdpart_LIBRARIES "")
message("alkaid_localization$ 配置第三方库")
include(cmake/thirdparty_lib.cmake)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "node/*_node.cpp")
list(REMOVE_ITEM  ALL_SRCS  ${NODE_SRCS})


add_executable(module_test_node node/module_test_node.cpp ${ALL_SRCS})
target_link_libraries(module_test_node ${catkin_LIBRARIES}   ${thirdpart_LIBRARIES})
add_test(module_test_node COMMAND module_test_node)