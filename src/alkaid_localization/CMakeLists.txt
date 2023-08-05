#cmake version
cmake_minimum_required(VERSION 3.13.0)
#project name
project(alkaid_localization)
#c++ config
set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
#roslib config
message("alkaid_localization$ 配置Ros库")
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

# selfpatry
file(GLOB_RECURSE ALL_SRCS "module/*.cpp" "pipe/*.cpp" "pub/*.cpp" "sub/*.cpp"  "tools/*.cpp"  "data/*.cpp")
file(GLOB_RECURSE NODE_SRCS "node/*_node.cpp"   )



# add_executable(module_test_node node/module_test_node.cpp ${ALL_SRCS})
# target_link_libraries(module_test_node ${catkin_LIBRARIES}  ${thirdparty_LIBRARIES})
# add_test(module_test_node COMMAND module_test_node)


add_executable(test_node node/test_node.cpp ${ALL_SRCS})
target_link_libraries(test_node ${catkin_LIBRARIES}  ${thirdparty_LIBRARIES})
# add_test(test_node COMMAND test_node)