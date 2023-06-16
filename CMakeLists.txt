#cmake 版本
cmake_minimum_required(VERSION 3.5)
#工程名称
project(AlkaidQuadrotor)

# 编译配置
set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

#文件存放 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)##bin 原则上只存*_node作为启动入口
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)#.so
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)#.a

#第三方库

include(cmake/thirdpartlib.cmake)

#src
add_subdirectory(src)
