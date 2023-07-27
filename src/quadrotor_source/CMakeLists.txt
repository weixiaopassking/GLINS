cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_source)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

#ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
include_directories( ${catkin_INCLUDE_DIRS})
catkin_package()


#message(${PROJECT_SOURCE_DIR})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)##bin 原则上只存*_node作为启动入口
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)#.so
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)#.a




set(project_path ${CMAKE_CURRENT_SOURCE_DIR})
set(thirdparty_LIBRARIES)
include(cmake/thirdparty_libs.cmake)



#src
add_subdirectory(src)

# set(cmake_path ${catkin_path})
# string(APPEND cmake_file_path "cmake/thirdparty_libs.cmake")
# message("cmakefile" ${cmake_file_path})


# add_executable(vis node/vis_node.cpp )
# target_link_libraries(vis ${catkin_LIBRARIES}  ${thirdparty_LIBRARIES} )


