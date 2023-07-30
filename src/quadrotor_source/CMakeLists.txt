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


#第三方库
set(project_path ${CMAKE_CURRENT_SOURCE_DIR})
configure_file(${project_path}/src/common/project_path.h.in  ${project_path}/src/common/project_path.h)
set(project_name ${PROJECT_NAME})
set(thirdparty_LIBRARIES)
include(cmake/thirdparty_libs.cmake)



#src
add_subdirectory(src)

# get_directory_property (inc_dirs INCLUDE_DIRECTORIES)
# foreach( dir ${inc_dirs} )
#     message(  "包含的目录: ${dir}" )
# endforeach()




