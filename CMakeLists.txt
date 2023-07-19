#cmake 版本
cmake_minimum_required(VERSION 3.5)
#工程名称
project(AlkaidQuadrotor)

# 编译配置
set(DEFAULT_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)

#工程目录
set(PROJECT_PATH ${PROJECT_SOURCE_DIR})
configure_file(cmake/project_path.h.in  ../src/common/project_path.h)


#文件存放 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)##bin 原则上只存*_node作为启动入口
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)#.so
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)#.a


include(cmake/thirdpart_libs.cmake)

#src
add_subdirectory(src)
